"""Milestone 3 — the task stops being about joints.

    python examples/03_osc.py                 # viewer, full law
    python examples/03_osc.py --break-it      # no null-space term
    python examples/03_osc.py -b l            # no Λ(q)
    python examples/03_osc.py -b h            # no h(q, q̇)
    python examples/03_osc.py --headless      # prints the errors

The arm is asked for an end-effector *pose*, not a set of angles, and it is
asked for it twice a second: the goal pose's frame, oscillated out and back on a
minimum-jerk profile. Seven joints chasing six numbers leaves one degree of
freedom over, and what happens to that spare joint is the whole lesson.

What you are looking at:

    yellow          x_d − x, from the gripper to where it was asked to be. The
                    only arrow on the ladder that is a length, and the one that
                    should shrink to nothing.
    pink            Jᵀ F — the task wrench priced through Λ and pushed back down
                    the arm, drawn both at the gripper and at each joint.
    green           N τ₀ at each joint — the posture term. It is supposed to do
                    nothing whatsoever to the tip, which is what makes it
                    impossible to see any other way.
    wide orange     h(q, q̇), the same arrow rung 0 drew.
    blue ellipsoid  √det(J Jᵀ) at the gripper: a ball where the arm is dextrous,
                    flattening towards a disc as a direction is lost.

The break-it modes, and the headless numbers they produce. `--settle` is the
cleanest way to see the null-space one, because a settled arm has no tracking
error left to confuse it with:

    (none)      3.25 mm of peak tip error while oscillating, and the posture
                lagging by up to 14° at a second a leg. Settled: 0.00 mm,
                0.000°, and 0.00° of posture deviation — exactly where it was
                put, in both spaces at once.
    --break-it  no N τ₀. Settled, the tip is still on the pose to 0.08 mm and
                the arm is **98.48° away from the posture it was given**, having
                drifted there through the one direction the task does not
                constrain. Nothing is wrong with the tracking: the elbow simply
                went wherever Coriolis and model error pushed it.
                It costs tracking too, second hand — oscillating, peak error
                goes 3.25 -> 23.00 mm, because the pose it drifts into is a less
                dextrous one. Watch √det(J Jᵀ) fall from 0.078 to 0.045 as it
                goes.
    -b l        no Λ(q), which is Jacobian-transpose control. Against a static
                pose it costs nothing, because Λ is a mass and a settled arm has
                no acceleration to price: 0.00 mm either way. Against a moving
                one it costs a factor of 8.4 at every speed tried — here 3.25 mm
                becomes 27.29 mm.
    -b h        no h(q, q̇). 15.45 mm of steady droop, for the same reason rung 0
                existed.

K_p here is in newtons per metre — it is the gripper's stiffness, and it means
the same thing in every pose because Λ absorbed the arm. Tripling it thirds the
error: 200 N/m gives 8.83 mm, 600 gives 3.26, 1800 gives 1.14.

One number worth not being surprised by: a step to a pose 20 cm away at 600 N/m
asks for 120 N, which through Jᵀ is several hundred newton-metres against
actuators that stop at 80. The demand is clipped and the arm arrives anyway —
but `--settle` reports the clipping, because a law whose first instant is
impossible is worth knowing about.
"""

import argparse
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from mujoManipulation.controller.osc import OperationalSpaceControl
from mujoManipulation.env import MuJoCoEnv
from mujoManipulation.robot.panda import FrankaPanda
from mujoManipulation.viz.primitives import Frame

XML = os.path.join("mujoManipulation", "assets", "panda", "franka_panda.xml")

# The pose the arm is sent to, relative to HOME. The server offers the same one
# under the name "reach out".
GOAL = np.array([0.6, 0.35, 0.0, 0.5, 0.0, -0.5, 1.0])

# Task-space gains, in N/m and N·s/m. Six numbers would work; one is honest,
# because after Λ every direction is the same system.
K_P, K_D = 600.0, 50.0

# Joint-space, because the posture task is.
K_P_NULL, K_D_NULL = 12.0, 7.0


def minimum_jerk(tau: float):
    """s, ṡ of the quintic that leaves and arrives at rest.

    The same polynomial the server uses, written out again rather than imported:
    an example has to run without the web layer, and this is four lines.
    """
    if tau <= 0.0:
        return 0.0, 0.0
    if tau >= 1.0:
        return 1.0, 0.0
    return (10 * tau**3 - 15 * tau**4 + 6 * tau**5,
            30 * tau**2 - 60 * tau**3 + 30 * tau**4)


def trajectory(t: float, home: np.ndarray, duration: float):
    """Out to the goal and back, forever, with a desired joint velocity."""
    phase = (t / duration) % 2.0
    tau = phase if phase <= 1.0 else 2.0 - phase
    direction = 1.0 if phase <= 1.0 else -1.0
    s, sd = minimum_jerk(tau)
    return home + GOAL * s, GOAL * sd * direction / duration


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "-b", "--break-it", nargs="?", const="n", choices=["n", "l", "h"],
        help="drop one term: n (the null-space posture), l (the operational-space "
             "inertia Λ), h (the bias). Bare means n.",
    )
    parser.add_argument("--settle", action="store_true",
                        help="go to the pose once and hold, instead of oscillating")
    parser.add_argument("--duration", type=float, default=1.0,
                        help="seconds for one leg of the motion")
    parser.add_argument("--headless", action="store_true",
                        help="run without opening a window")
    parser.add_argument("--seconds", type=float, default=8.0,
                        help="simulated seconds")
    args = parser.parse_args()

    broke = args.break_it
    xml = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), XML)

    with MuJoCoEnv(xml, render=not args.headless) as env:
        panda = FrankaPanda(env.model, env.data)
        panda.go_home()

        controller = OperationalSpaceControl(
            panda, K_p=K_P, K_d=K_D, K_p_null=K_P_NULL, K_d_null=K_D_NULL,
            use_nullspace=broke != "n",
            use_lambda=broke != "l",
            use_bias=broke != "h",
        )

        home = panda.HOME.copy()
        label = {None: "full OSC", "n": "no null-space term  (break it)",
                 "l": "no Λ(q)  (break it)", "h": "no h(q, q̇)  (break it)"}[broke]
        print(f"controller           : {label}")
        print(f"task gains           : K_p={K_P:g} N/m  K_d={K_D:g} N·s/m")
        print(f"posture gains        : K_p={K_P_NULL:g}  K_d={K_D_NULL:g}  (joint space)")
        print(f"task                 : "
              + ("hold the goal pose" if args.settle
                 else f"oscillate to it and back, {args.duration:g} s a leg"))

        n_steps = int(args.seconds / env.dt)
        started = time.time()
        settle_after = args.duration if not args.settle else 0.5

        pos_err = np.zeros(n_steps)
        rot_err = np.zeros(n_steps)
        posture_err = np.zeros((n_steps, panda.n_arm))
        manip = np.zeros(n_steps)
        pieces = np.zeros((n_steps, 3, panda.n_arm))
        ever_saturated = np.zeros(panda.n_arm, dtype=bool)
        recorded = 0

        for step in range(n_steps):
            if not env.is_running():
                break

            t = step * env.dt
            if args.settle:
                q_d, qd_d = home + GOAL, np.zeros(panda.n_arm)
            else:
                q_d, qd_d = trajectory(t, home, args.duration)

            tau, primitives = controller.update(q_d, qd_d)
            # The pose being chased, and the pose the gripper is at. Facts about
            # the task rather than terms of the law, so the caller draws them --
            # the server does exactly the same thing for every rung.
            primitives = primitives + [
                Frame(pos=controller.target_pos, mat=controller.target_mat,
                      length=0.15, width=0.011),
                Frame(pos=panda.ee_pos, mat=panda.ee_mat, length=0.10, width=0.005),
            ]
            panda.set_arm_torque(tau)
            env.step()
            env.sync(primitives)

            pos_err[step] = np.linalg.norm(controller.error[:3]) * 1000
            rot_err[step] = np.degrees(np.linalg.norm(controller.error[3:]))
            posture_err[step] = np.degrees(panda.arm_q - q_d)
            manip[step] = controller.manipulability
            pieces[step] = (controller.tau_task, controller.tau_null,
                            controller.tau_bias)
            ever_saturated |= controller.saturated
            recorded = step + 1

            if not args.headless:
                # keep the viewer roughly real time so it is watchable
                ahead = (step + 1) * env.dt - (time.time() - started)
                if ahead > 0:
                    time.sleep(ahead)

        report(pos_err[:recorded], rot_err[:recorded], posture_err[:recorded],
               manip[:recorded], pieces[:recorded], int(settle_after / env.dt),
               ever_saturated)

    return 0


# --- measurement, not control --------------------------------------------


def vec(values, places: int = 2) -> str:
    return "[" + " ".join(f"{v:7.{places}f}" for v in values) + "]"


def report(pos_err, rot_err, posture_err, manip, pieces, skip, ever_saturated) -> None:
    if len(pos_err) == 0:
        return
    after = slice(min(skip, len(pos_err) - 1), None)

    print("--- the tip " + "-" * 41)
    print(f"peak position error  : {pos_err[after].max():.2f} mm")
    print(f"final                : {pos_err[-1]:.2f} mm, {rot_err[-1]:.3f}°")
    print("--- the spare joint " + "-" * 33)
    print(f"worst deviation      : {np.abs(posture_err[-1]).max():.2f}° from the "
          f"posture it was given")
    print(f"per joint            : {vec(np.abs(posture_err[-1]))} deg")
    print("--- the margin " + "-" * 38)
    print(f"manipulability       : {manip.min():.4f} .. {manip.max():.4f}  "
          f"(√det(J Jᵀ), 0 at a singularity)")
    print("--- the three terms, peak magnitude " + "-" * 17)
    for i, name in enumerate(("Jᵀ F", "N τ₀", "h(q, q̇)")):
        print(f"{name:<21}: {vec(np.abs(pieces[:, i, :]).max(axis=0), 1)} N·m")
    if ever_saturated.any():
        hit = (np.flatnonzero(ever_saturated) + 1).tolist()
        print(f"clipped              : joint(s) {hit} — asked for more torque "
              f"than the actuator has")


if __name__ == "__main__":
    raise SystemExit(main())
