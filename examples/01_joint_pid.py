"""Milestone 1 — the arm goes where you ask, and you can see what moves it.

    python examples/01_joint_pid.py                   # viewer, full PID
    python examples/01_joint_pid.py --break-it        # no D term: it rings
    python examples/01_joint_pid.py --break-it i      # no I term: it droops
    python examples/01_joint_pid.py --break-it windup # the integral winds up
    python examples/01_joint_pid.py --headless        # no window, prints metrics

The reference is a square wave: every `--hold` seconds the arm is asked to jump
between its home pose and an offset pose. One input, and all three terms end up
visibly doing their separate jobs.

What you are looking at:

    pale skeleton   the pose the reference describes, drawn through the joint
                    anchors — the arm you asked for. The gap to the real arm is e.
    thin blue       K_p e, the spring. Longest right after the step.
    green           K_i ∫e dt, the memory. Grows while the error persists and
                    then stays, holding the load P could not.
    wide faint      K_d ė, the damper. Only exists while the error is moving, so
                    it flashes on the approach and vanishes at rest.

Arrows sit at the joint they act on, along that joint's own rotation axis, at
the same 40 N·m per metre as the gravity-comp demo — so a term here is directly
comparable to g(q) there.

Note what is *not* here: no gravity compensation. This controller is told
nothing about the arm's weight, and the I term beats gravity anyway, by growing
until it does — watch the green arrows settle at roughly the orange ones from
milestone 0. `--gravity-comp` adds that feedforward on top instead, which is the
cheaper answer to the same problem: the steady-state droop goes away without
anyone having to integrate their way out of it.

The three break-it modes, and the headless numbers they produce:

    (none)      peak overshoot 47.4%, settled in 2.314 s, steady-state error
                under a tenth of a degree on every joint. The integral ends up
                holding [-10.4, 17.3, 1.3] N·m on joints 2, 4 and 6, against a
                g(q) of [-10.7, 17.4, 1.3] N·m at that pose — it found gravity
                to 2% without being told gravity exists, the whole trick.
    --break-it  no D. Overshoot 276%, never settles, and all seven joints slam
                into their torque limits — joint 7 ends the run 169° out.
                Nothing is left to absorb the energy P keeps pumping in: the XML
                gives these joints 0.1 N·m·s/rad of damping, and that is all.
    -b i        no I. Overshoot drops to 35% — the integral was paying for some
                of it — and then the arm parks 7.08° and 8.53° short on joints 2
                and 4 and stays there, so the settling band is never reached at
                all. Those two numbers are g(q)/K_p exactly: 14.83/120 and
                17.86/120 rad. That is where P stalls, and waiting does not fix
                it.
    -b windup   no anti-windup, against a target past joint 4's limit so that
                the demand cannot be met. Joint 4's integral reaches -306 N·m on
                an ±80 N·m channel (-35 N·m with anti-windup on), and all of it
                has to be unwound before the joint can come back: the return
                home never completes, where the same run with anti-windup gets
                there in 2.4 s. `--push-limit` is that run — the same
                unreachable target, anti-windup left alone.
"""

import argparse
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from mujoManipulation.controller.gravity_comp import GravityCompensation
from mujoManipulation.controller.pid import JointSpacePID
from mujoManipulation.env import MuJoCoEnv
from mujoManipulation.robot.panda import FrankaPanda

XML = os.path.join("mujoManipulation", "assets", "panda", "franka_panda.xml")

# Where the arm is asked to go, relative to HOME, in radians. Moves every joint
# that can move the tip, and stays inside the joint limits.
STEP = np.array([0.6, 0.35, 0.0, 0.5, 0.0, -0.5, 1.0])

# How far past joint 4's limit the windup demo reaches, in radians.
PAST_LIMIT = 0.4

# Gains.
#
# K_p is a stiffness, per joint: joint 1 carries the whole arm, joint 7 carries
# a gripper, so they do not want the same number. Pick those by hand, then let
# the other two follow from a model. Treat one joint on its own as M q̈ = τ with
# M the mass-matrix diagonal at the home pose — mj_fullM gives
# [1.86, 2.45, 1.65, 1.52, 0.53, 0.39, 0.20] kg·m² — and the closed loop is
#
#     M s³ + K_d s² + K_p s + K_i = 0
#
# Ask for all three roots at the same place, s = −a. Expanding M(s + a)³ gives
#
#     K_d = 3aM        K_p = 3a²M        K_i = a³M      so   a = √(K_p / 3M)
#
# which turns the K_p you chose into the other two with no tuning. The numbers
# below are that, rounded. It is only an estimate: the arm is coupled and these
# per-joint models pretend it is not, which is most of why joint 2 still
# overshoots 43% — joints 1 and 4 swinging through are a disturbance its own
# model never mentioned. Milestone 2 is where that excuse runs out.
#
# One trap if you retune by hand: K_d and K_i do not move independently. Raising
# K_d alone slows the response while the integral keeps winding at the same
# rate, so overshoot goes *up* — tripling K_d here takes joint 1 from 23% to
# 31%. The damping knob is not a "less overshoot" knob unless K_i follows it.
K_P = np.array([120.0, 120.0, 120.0, 120.0, 50.0, 20.0, 10.0])
K_D = np.array([26.0, 30.0, 24.0, 23.0, 9.0, 4.8, 2.5])
K_I = np.array([185.0, 160.0, 195.0, 205.0, 95.0, 28.0, 14.0])

SETTLE_BAND = np.radians(1.0)  # "arrived" means inside one degree, and staying


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "-b", "--break-it",
        nargs="?",
        const="d",
        choices=["d", "i", "windup"],
        help="drop one term: d (the damper), i (the memory), windup (the "
             "anti-windup, against an unreachable target). Bare means d.",
    )
    parser.add_argument(
        "--push-limit",
        action="store_true",
        help="aim joint 4 past its own limit, anti-windup left on — the run to "
             "compare --break-it windup against",
    )
    parser.add_argument(
        "--gravity-comp",
        action="store_true",
        help="add the milestone-0 g(q) feedforward on top of the PID",
    )
    parser.add_argument(
        "--headless", action="store_true", help="run without opening a window"
    )
    parser.add_argument(
        "--seconds", type=float, default=14.0, help="simulated seconds"
    )
    parser.add_argument(
        "--hold", type=float, default=3.5, help="seconds between reference steps"
    )
    args = parser.parse_args()

    broke = args.break_it
    # Windup is a property of a demand that cannot be met, so it needs the
    # unreachable target to have anything to show.
    unreachable = args.push_limit or broke == "windup"
    xml = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), XML)

    with MuJoCoEnv(xml, render=not args.headless) as env:
        panda = FrankaPanda(env.model, env.data)
        panda.go_home()

        pid = JointSpacePID(
            panda,
            K_p=K_P,
            K_i=K_I,
            K_d=K_D,
            dt=env.dt,  # the step the loop below actually runs at
            d_enabled=broke != "d",
            i_enabled=broke != "i",
            anti_windup=broke != "windup",
        )
        gravity = GravityCompensation(panda) if args.gravity_comp else None
        # Same class, used as an instrument rather than a controller: at the end
        # of the first reach we ask it what gravity was doing, purely so the
        # report can hold it up against what the integral had accumulated.
        probe = gravity if gravity is not None else GravityCompensation(panda)

        home = panda.HOME.copy()
        target = home + STEP
        if unreachable:
            j4_min = env.model.jnt_range[panda.arm_joint_ids[3], 0]
            target[3] = j4_min - PAST_LIMIT

        label = {
            None: "full PID",
            "d": "no D term  (break it)",
            "i": "no I term  (break it)",
            "windup": "no anti-windup  (break it)",
        }[broke]
        print(f"controller           : {label}")
        print(f"feedforward          : {'g(q)' if gravity else 'none'}")
        print(f"dt                   : {pid.dt} s  (model timestep {env.dt} s)")
        print(f"reference            : home ↔ target every {args.hold:g} s")
        print(f"step to target       : {vec(np.degrees(target - home), 1)} deg")
        if unreachable:
            print(f"joint 4 target       : {np.degrees(PAST_LIMIT):.0f}° past its "
                  f"own limit — the arm cannot get there")

        n_steps = int(args.seconds / env.dt)
        started = time.time()

        q_trace = np.zeros((n_steps, panda.n_arm))
        tau_trace = np.zeros((n_steps, panda.n_arm))
        ever_saturated = np.zeros(panda.n_arm, dtype=bool)
        recorded = 0
        last_outbound = int(args.hold / env.dt) - 1
        integral_at_rest = gravity_at_rest = None

        for step in range(n_steps):
            if not env.is_running():
                break

            t = step * env.dt
            outbound = int(t / args.hold) % 2 == 0
            q_ref = target if outbound else home

            tau, primitives = pid.update(q_ref)
            if gravity is not None:
                tau_g, primitives_g = gravity.update()
                tau = tau + tau_g
                primitives = primitives + primitives_g

            panda.set_arm_torque(tau)
            env.step()
            env.sync(primitives)

            q_trace[step] = panda.arm_q
            tau_trace[step] = tau
            ever_saturated |= pid.saturated
            recorded = step + 1

            if step == last_outbound:
                integral_at_rest = pid.i_term.copy()
                gravity_at_rest = probe.gravity_torque()

            if not args.headless:
                # keep the viewer roughly real time so it is watchable
                ahead = (step + 1) * env.dt - (time.time() - started)
                if ahead > 0:
                    time.sleep(ahead)

        report(
            q_trace[:recorded],
            tau_trace[:recorded],
            home,
            target,
            ever_saturated,
            panda,
            env.dt,
            args.hold,
            integral_at_rest,
            gravity_at_rest,
            feedforward=gravity is not None,
        )

    return 0


# --- measurement, not control --------------------------------------------


def vec(values, places: int = 2) -> str:
    """Fixed-point, one column per joint — numpy's default flips to scientific
    notation as soon as one entry is near zero, which makes seven joints
    unreadable."""
    return "[" + " ".join(f"{v:7.{places}f}" for v in values) + "]"


def segment(trace, dt, hold, index):
    """Rows of `trace` belonging to the `index`-th hold interval."""
    lo, hi = int(index * hold / dt), int((index + 1) * hold / dt)
    return trace[lo:min(hi, len(trace))]


def settling_time(err, moved, dt):
    """Seconds until every stepped joint is inside the band and stays there.

    `None` if it never managed it — which is the honest answer, and the one
    --break-it d produces.
    """
    outside = np.any(np.abs(err[:, moved]) > SETTLE_BAND, axis=1)
    last = np.flatnonzero(outside)
    if not last.size:
        return 0.0
    if last[-1] == len(outside) - 1:
        return None
    return (last[-1] + 1) * dt


def report(q_trace, tau_trace, home, target, ever_saturated, panda, dt, hold,
           integral_at_rest=None, gravity_at_rest=None, feedforward=False) -> None:
    if len(q_trace) == 0:
        return

    out = segment(q_trace, dt, hold, 0)
    step = target - home
    moved = step != 0.0
    err = out - target

    # Overshoot: how far past the target it went, in the direction it travelled.
    overshoot = np.max(np.sign(step) * err, axis=0).clip(min=0.0)
    overshoot = np.where(moved, overshoot, 0.0)
    worst = int(np.argmax(np.where(moved, overshoot / np.abs(np.where(moved, step, 1)), 0)))
    final = np.abs(err[-1])
    settle = settling_time(err, moved, dt)
    peak_tau = np.abs(segment(tau_trace, dt, hold, 0)).max(axis=0)

    print("--- reaching the target " + "-" * 29)
    print(f"peak overshoot       : {100 * overshoot[worst] / abs(step[worst]):.1f}% "
          f"of its step  (joint {worst + 1}, {np.degrees(overshoot[worst]):.2f}°)")
    print(f"settling time        : "
          + ("never settled within 1°" if settle is None else f"{settle:.3f} s  (to 1°)"))
    print(f"steady-state error   : {vec(np.degrees(final))} deg")
    print(f"peak |torque|        : {vec(peak_tau, 1)} N·m")
    print(f"torque limits        : {vec(panda.arm_torque_limits[:, 1], 1)} N·m")
    if ever_saturated.any():
        hit = (np.flatnonzero(ever_saturated) + 1).tolist()
        print(f"clipped              : joint(s) {hit} — asked for more torque "
              f"than the actuator has")

    # What the integral grew into, against what gravity was asking for at that
    # pose. When the arm did settle, those two should be the same number, and
    # nothing told the integrator about gravity — it worked the load out from a
    # persistent error and nothing else. When the arm did not settle, the pair
    # is still worth printing: it is where the integral went instead.
    if integral_at_rest is not None and not feedforward:
        tag = "  <- nobody told it about gravity" if settle is not None else ""
        print(f"integral at rest     : {vec(integral_at_rest)} N·m")
        print(f"g(q) at that pose    : {vec(gravity_at_rest)} N·m{tag}")

    back = segment(q_trace, dt, hold, 1)
    if len(back) > int(0.9 * hold / dt):
        home_err = back - home
        settle_back = settling_time(home_err, moved, dt)
        print("--- and coming back " + "-" * 33)
        print(f"settling time        : "
              + ("never settled within 1°" if settle_back is None
                 else f"{settle_back:.3f} s  (to 1°)"))
        print(f"error on arrival     : {vec(np.degrees(np.abs(home_err[-1])))} deg")


if __name__ == "__main__":
    raise SystemExit(main())
