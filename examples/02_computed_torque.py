"""Milestone 2 — the model goes into the controller, and the gains stop lying.

    python examples/02_computed_torque.py              # viewer, full law
    python examples/02_computed_torque.py --break-it    # no M(q)
    python examples/02_computed_torque.py -b c          # no Coriolis term
    python examples/02_computed_torque.py -b c --speed 0.25   # ...and now it is fine
    python examples/02_computed_torque.py -b g          # no gravity term
    python examples/02_computed_torque.py --headless    # no window, prints the error

The reference is a smooth joint-space oscillation, q_d = home + A(1 − cos ωt)/2,
differentiated twice analytically so q̇_d and q̈_d go in as feedforward. Unlike
milestone 1's square wave it has no corners, and `--speed` scales ω — which is
the knob the Coriolis lesson turns.

What you are looking at:

    pale skeleton   the pose the reference asks for.
    violet          M(q) q̈_cmd — what it costs to accelerate *this* arm from
                    *this* pose. The command barely changes through the stroke;
                    this arrow does, because M(q) does. That is the quantity PID
                    had to absorb into its gains.
    cyan            C(q, q̇) q̇ — the velocity term. Zero at the turnaround,
                    biggest mid-stroke, and the whole arrow scales with q̇².
    wide orange     g(q) — rung 0's arrow, unchanged.

Two gains for the entire arm, where milestone 1 needed seven pairs: after the
cancellation every joint obeys ë + K_d ė + K_p e = 0, so K_p = ω² and
K_d = 2ζω are a specification (how fast, how damped) rather than a tuning.

`--pid` is the comparison that makes the case: the same stroke, the same plant,
driven by milestone 1's law and milestone 1's seven pairs of gains instead.

    computed torque   0.372° peak, 0.256° rms
    PID               17.382° peak, 9.953° rms        — 47x on the peak

PID is not badly tuned; it is structurally behind. It has no q̈_d to feed
forward, so every newton-metre it applies has to be paid for by an error that
already happened, and it has no M(q), so the error it is correcting is priced
wrong and differently at each pose. Note too that joint 7 is never commanded to
move at all and still ends up 17° out under PID: that is the rest of the arm
dragging it, a disturbance PID cannot see and this law cancels by construction.

The break-it modes, and the headless numbers they produce:

    (none)      0.372° peak error with the wrist passing 2.8 rad/s, and nothing
                saturating. What is left is model error, not gain error: the
                XML's 0.1 N·m·s/rad of joint damping is a passive force, lives in
                qfrc_passive, and is not part of h(q, q̇), so it is never
                cancelled. At `--speed 0.25` the peak is 0.075°.
    --break-it  no M(q): the acceleration command is sent as though every joint
                had unit inertia. Peak error goes to 4.223°, 11x worse, the worst
                joint moves from 1 to 3, and joint 6 starts hitting its torque
                limit. The run prints what was being ignored — over this one
                stroke joint 2's own inertia swings 2.45 -> 3.51 kg·m² (43%), so
                no fixed gain is right at both ends, and the largest off-diagonal
                reaches 1.84 kg·m², bigger than four of the seven diagonals, so
                "unit inertia" gets the coupling wrong as well as the scale.
    -b c        no C(q, q̇) q̇. Joint 5 goes from 0.240° to 0.957° — and at
                `--speed 0.25` from 0.046° to 0.103°. So the term's *cost* falls
                from 0.72° to 0.06°, a factor of 12 for a factor of 4 in speed,
                which is the q̇² scaling showing up in the one place it can. This
                is why a slow robot can ignore Coriolis and a fast one cannot.
    -b g        no g(q). Joint 4 sits 3.3° low (rms) and stays there through the
                whole stroke: a bias the PD re-fights at every pose with nothing
                accumulating to help it. Milestone 1's I term is what used to.
"""

import argparse
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from mujoManipulation.controller.computed_torque import ComputedTorque
from mujoManipulation.controller.pid import JointSpacePID
from mujoManipulation.env import MuJoCoEnv
from mujoManipulation.robot.panda import FrankaPanda

XML = os.path.join("mujoManipulation", "assets", "panda", "franka_panda.xml")

# Oscillation amplitude per joint, in radians, measured from HOME. Joints 1, 2,
# 4 and 5 are the ones that swing the arm's mass around, which is what makes
# M(q) visibly change and the Coriolis coupling worth anything.
AMPLITUDE = np.array([0.9, 0.5, 0.0, 0.7, 0.6, 0.5, 0.0])

# Base frequency of the stroke, in Hz. `--speed` multiplies it.
FREQUENCY = 1.0

# The error dynamics, specified rather than tuned: K_p = ω², K_d = 2ζω gives
# ë + K_d ė + K_p e = 0 with a settling time of about 4/ω. One pair for all
# seven joints, because after the cancellation they are the same system.
OMEGA = 20.0   # rad/s — how fast an error dies
ZETA = 1.0     # critically damped, so it dies without ringing
K_P = OMEGA**2
K_D = 2.0 * ZETA * OMEGA

# Milestone 1's gains, for `--pid`. Copied rather than imported so that each
# example stays readable and runnable on its own — and so that the seven pairs
# are sitting right here next to the two above, which is the comparison.
PID_K_P = np.array([120.0, 120.0, 120.0, 120.0, 50.0, 20.0, 10.0])
PID_K_D = np.array([26.0, 30.0, 24.0, 23.0, 9.0, 4.8, 2.5])
PID_K_I = np.array([185.0, 160.0, 195.0, 205.0, 95.0, 28.0, 14.0])


def trajectory(t: float, home: np.ndarray, omega: float):
    """q_d, q̇_d, q̈_d for the stroke, differentiated by hand.

        q_d  = home + A (1 − cos ωt) / 2
        q̇_d  =        A ω sin ωt / 2
        q̈_d  =        A ω² cos ωt / 2

    The (1 − cos) shape rather than a sine so that at t = 0 both the position
    and the velocity match the arm's actual state — the run starts with zero
    error, and every number after that is tracking, not a startup transient.
    """
    half = AMPLITUDE / 2.0
    return (
        home + half * (1.0 - np.cos(omega * t)),
        half * omega * np.sin(omega * t),
        half * omega**2 * np.cos(omega * t),
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "-b", "--break-it",
        nargs="?",
        const="m",
        choices=["m", "c", "g"],
        help="drop one term: m (the inertia M(q)), c (the Coriolis term), "
             "g (gravity). Bare means m.",
    )
    parser.add_argument(
        "--pid", action="store_true",
        help="drive the same stroke with milestone 1's PID instead — no model, "
             "seven pairs of gains",
    )
    parser.add_argument(
        "--speed", type=float, default=1.0,
        help="multiply the stroke frequency — the Coriolis term scales with its "
             "square",
    )
    parser.add_argument(
        "--headless", action="store_true", help="run without opening a window"
    )
    parser.add_argument(
        "--seconds", type=float, default=8.0, help="simulated seconds"
    )
    args = parser.parse_args()

    broke = args.break_it
    omega = 2.0 * np.pi * FREQUENCY * args.speed
    xml = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), XML)

    with MuJoCoEnv(xml, render=not args.headless) as env:
        panda = FrankaPanda(env.model, env.data)
        panda.go_home()

        if args.pid:
            controller = JointSpacePID(
                panda, K_p=PID_K_P, K_i=PID_K_I, K_d=PID_K_D, dt=env.dt
            )
        else:
            controller = ComputedTorque(
                panda,
                K_p=K_P,
                K_d=K_D,
                use_inertia=broke != "m",
                use_coriolis=broke != "c",
                use_gravity=broke != "g",
            )

        home = panda.HOME.copy()
        moved = AMPLITUDE != 0.0

        label = {
            None: "full computed torque",
            "m": "no M(q)  (break it)",
            "c": "no Coriolis term  (break it)",
            "g": "no g(q)  (break it)",
        }[broke]
        peak_qd = np.abs(AMPLITUDE / 2.0 * omega)
        print(f"controller           : "
              f"{'milestone 1 PID, no model' if args.pid else label}")
        if args.pid:
            print(f"gains                : seven hand-picked pairs, K_p="
                  f"{vec(PID_K_P, 0)}")
        else:
            print(f"error dynamics       : K_p={K_P:g}  K_d={K_D:g}   "
                  f"(omega={OMEGA:g} rad/s, zeta={ZETA:g})")
        print(f"stroke               : {FREQUENCY * args.speed:g} Hz, "
              f"amplitude {vec(np.degrees(AMPLITUDE), 1)} deg")
        print(f"peak |q̇_d|           : {vec(peak_qd)} rad/s")

        n_steps = int(args.seconds / env.dt)
        started = time.time()

        err = np.zeros((n_steps, panda.n_arm))
        pieces = np.zeros((n_steps, 3, panda.n_arm))
        m_diag = np.zeros((n_steps, panda.n_arm))
        m_coupling = np.zeros(n_steps)
        ever_saturated = np.zeros(panda.n_arm, dtype=bool)
        recorded = 0

        for step in range(n_steps):
            if not env.is_running():
                break

            t = step * env.dt
            q_d, qd_d, qdd_d = trajectory(t, home, omega)

            if args.pid:
                # The PID knows a position and a velocity, and that is all it
                # can be given — there is nowhere to put q̈_d in its law.
                tau, primitives = controller.update(q_d, qd_d)
            else:
                tau, primitives = controller.update(q_d, qd_d, qdd_d)

            panda.set_arm_torque(tau)
            env.step()
            env.sync(primitives)

            err[step] = panda.arm_q - q_d
            if not args.pid:
                pieces[step] = (
                    controller.tau_inertial,
                    controller.tau_coriolis,
                    controller.tau_gravity,
                )
                M = controller.inertia
                m_diag[step] = np.diag(M)
                m_coupling[step] = np.abs(M - np.diag(np.diag(M))).max()
            ever_saturated |= controller.saturated
            recorded = step + 1

            if not args.headless:
                # keep the viewer roughly real time so it is watchable
                ahead = (step + 1) * env.dt - (time.time() - started)
                if ahead > 0:
                    time.sleep(ahead)

        report(err[:recorded], pieces[:recorded], m_diag[:recorded],
               m_coupling[:recorded], moved, ever_saturated, model=not args.pid)

    return 0


# --- measurement, not control --------------------------------------------


def vec(values, places: int = 2) -> str:
    """Fixed-point, one column per joint — numpy's default flips to scientific
    notation as soon as one entry is near zero."""
    return "[" + " ".join(f"{v:7.{places}f}" for v in values) + "]"


def report(err, pieces, m_diag, m_coupling, moved, ever_saturated,
           model: bool = True) -> None:
    if len(err) == 0:
        return

    deg = np.degrees(np.abs(err))
    peak = deg.max(axis=0)
    rms = np.sqrt((deg**2).mean(axis=0))
    worst = int(np.argmax(peak))

    print("--- tracking " + "-" * 40)
    print(f"peak |error|         : {vec(peak, 3)} deg")
    print(f"rms  |error|         : {vec(rms, 3)} deg")
    print(f"worst joint          : {worst + 1}, peak {peak[worst]:.3f}°")

    if not model:
        # Everything below needs the model in the loop. Its absence is the
        # entire content of --pid.
        if ever_saturated.any():
            hit = (np.flatnonzero(ever_saturated) + 1).tolist()
            print(f"clipped              : joint(s) {hit}")
        return

    inertial, coriolis, gravity = (np.abs(pieces[:, i, :]).max(axis=0) for i in range(3))

    # What M(q) did over the run. This is the thing PID had to absorb into its
    # gains and could not: not a constant, not even diagonal.
    lo, hi = m_diag.min(axis=0), m_diag.max(axis=0)
    swing = int(np.argmax(hi / np.maximum(lo, 1e-9)))
    below = int((hi < m_coupling.max()).sum())
    print("--- what the gains would have to absorb without M(q) " + "-" * 0)
    print(f"own inertia, lowest  : {vec(lo)} kg·m²")
    print(f"own inertia, highest : {vec(hi)} kg·m²")
    print(f"biggest swing        : joint {swing + 1}, {lo[swing]:.2f} -> "
          f"{hi[swing]:.2f} kg·m² ({100 * (hi[swing] / lo[swing] - 1):.0f}%) over "
          f"this one stroke — a fixed gain cannot be right at both ends")
    print(f"largest off-diagonal : {m_coupling.max():.2f} kg·m², bigger than "
          f"{below} of the 7 diagonals — pretending M is the identity gets the "
          f"coupling wrong too, not only the scale")
    print("--- the three terms, peak magnitude " + "-" * 17)
    print(f"M(q) q̈_cmd           : {vec(inertial, 1)} N·m")
    print(f"C(q, q̇) q̇            : {vec(coriolis, 1)} N·m")
    print(f"g(q)                 : {vec(gravity, 1)} N·m")
    if ever_saturated.any():
        hit = (np.flatnonzero(ever_saturated) + 1).tolist()
        print(f"clipped              : joint(s) {hit} — asked for more torque "
              f"than the actuator has")


if __name__ == "__main__":
    raise SystemExit(main())
