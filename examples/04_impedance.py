"""Milestone 4 — the arm yields, and you can choose how much.

    python examples/04_impedance.py                 # viewer: a hand pushes the gripper
    python examples/04_impedance.py --break-it      # no damper: it rings
    python examples/04_impedance.py -b h            # no h(q, q̇)
    python examples/04_impedance.py --stiffness 2400  # argue harder
    python examples/04_impedance.py --headless      # prints the deflection

The arm is asked to hold one pose, and for one second in the middle of the run
something pushes the gripper sideways with a known force. That push is the whole
experiment: every rung before this one would have fought it with whatever torque
it took, and this one is specified by *how much* it fights.

What you are looking at:

    blue helix      K e, the virtual spring, strung between the gripper and the
                    pose it was asked for. It stretches as the push lands and
                    coils back up as the arm returns — the behaviour of this
                    whole rung in one object.
    orange arrow    D ė at the gripper. Only there while something is moving.
    wide orange     h(q, q̇) at each joint.
    green           N τ₀ at each joint — the posture term.

Deflection is F/K, not approximately:

    stiffness   predicted   measured   settles in
      150 N/m     133 mm     132.4 mm    1.82 s
      600 N/m      33 mm      32.9 mm    0.83 s
     2400 N/m       8 mm       8.3 mm    0.24 s

which is Hooke's law coming out of a torque controller that was never told about
springs. Stiff is accurate and dangerous; soft is safe and vague; K is the dial
and it is in newtons per metre.

The break-it modes:

    --break-it  no D ė. The spring is still a spring and now it is a bell. It
                overshoots further on the way out — 113.7 mm of peak deflection
                against 76.4 mm with the damper — reverses direction 14 times
                after the push is released, and never comes back inside a
                millimetre of where it started, with only the XML's
                0.1 N·m·s/rad of joint damping to eventually stop it. Compare
                one reversal and 1.23 s with the damper in place.
    -b h        no h(q, q̇). The spring hangs from a sagging anchor.
    -b n        no N τ₀. The spare joint drifts, as it did in milestone 3.
"""

import argparse
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from mujoManipulation.controller.impedance import CartesianImpedance
from mujoManipulation.env import MuJoCoEnv
from mujoManipulation.robot.panda import FrankaPanda
from mujoManipulation.viz.primitives import Arrow, Frame

XML = os.path.join("mujoManipulation", "assets", "panda", "franka_panda.xml")

# When the push happens, how hard, and which way. Sideways, because that is the
# direction the arm has least to say about at the home pose.
PUSH_WINDOW = (1.0, 2.0)
PUSH_NEWTONS = 20.0
PUSH_DIRECTION = np.array([0.0, 1.0, 0.0])

# N·m per metre of arrow, shared with every rung; forces get twice that.
N_PER_METRE = 80.0
PUSH_RGBA = (0.95, 0.25, 0.3, 0.95)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "-b", "--break-it", nargs="?", const="d", choices=["d", "h", "n"],
        help="drop one term: d (the damper), h (the bias), n (the null-space "
             "posture). Bare means d.",
    )
    parser.add_argument("--stiffness", type=float, default=300.0,
                        help="task-space stiffness in N/m")
    parser.add_argument("--damping", type=float, default=None,
                        help="task-space damping in N·s/m (default: 2·√K)")
    parser.add_argument("--force", type=float, default=PUSH_NEWTONS,
                        help="how hard the push is, in newtons")
    parser.add_argument("--headless", action="store_true",
                        help="run without opening a window")
    parser.add_argument("--seconds", type=float, default=5.0,
                        help="simulated seconds")
    args = parser.parse_args()

    broke = args.break_it
    damping = args.damping if args.damping is not None else 2.0 * np.sqrt(args.stiffness)
    xml = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), XML)

    with MuJoCoEnv(xml, render=not args.headless) as env:
        panda = FrankaPanda(env.model, env.data)
        panda.go_home()
        ee_body = int(env.model.site_bodyid[panda.ee_site_id])

        controller = CartesianImpedance(
            panda, K_p=args.stiffness, K_d=damping,
            use_damping=broke != "d",
            use_bias=broke != "h",
            use_nullspace=broke != "n",
        )

        goal = panda.HOME.copy()
        label = {None: "full impedance", "d": "no damper  (break it)",
                 "h": "no h(q, q̇)  (break it)", "n": "no null-space term  (break it)"}[broke]
        print(f"controller           : {label}")
        print(f"stiffness            : {args.stiffness:g} N/m")
        print(f"damping              : {damping:g} N·s/m"
              + ("  — switched off" if broke == "d" else ""))
        print(f"push                 : {args.force:g} N along "
              f"{PUSH_DIRECTION.astype(int).tolist()}, "
              f"{PUSH_WINDOW[0]:g}–{PUSH_WINDOW[1]:g} s")
        print(f"predicted deflection : {1000 * args.force / args.stiffness:.1f} mm  (F/K)")

        n_steps = int(args.seconds / env.dt)
        started = time.time()

        ee = np.zeros((n_steps, 3))
        pushing = np.zeros(n_steps, dtype=bool)
        recorded = 0

        for step in range(n_steps):
            if not env.is_running():
                break

            t = step * env.dt
            push_now = PUSH_WINDOW[0] <= t < PUSH_WINDOW[1]
            force = args.force * PUSH_DIRECTION if push_now else np.zeros(3)

            # An external force at the gripper. On hardware this is a person;
            # here it is the one line of the experiment that is not the
            # controller, which is why it lives in the demo and not in it.
            env.data.xfrc_applied[ee_body] = 0.0
            env.data.xfrc_applied[ee_body, :3] = force

            tau, primitives = controller.update(goal)
            if push_now:
                primitives = primitives + [
                    Arrow(origin=panda.ee_pos, vec=force / N_PER_METRE,
                          width=0.015, rgba=PUSH_RGBA)
                ]
            primitives = primitives + [
                Frame(pos=controller.target_pos, mat=controller.target_mat,
                      length=0.15, width=0.011),
                Frame(pos=panda.ee_pos, mat=panda.ee_mat, length=0.10, width=0.005),
            ]

            panda.set_arm_torque(tau)
            env.step()
            env.sync(primitives)

            ee[step] = panda.ee_pos
            pushing[step] = push_now
            recorded = step + 1

            if not args.headless:
                ahead = (step + 1) * env.dt - (time.time() - started)
                if ahead > 0:
                    time.sleep(ahead)

        report(ee[:recorded], pushing[:recorded], env.dt, args)

    return 0


# --- measurement, not control --------------------------------------------


def report(ee, pushing, dt, args) -> None:
    if not pushing.any():
        return
    first = int(np.argmax(pushing))
    last = int(len(pushing) - np.argmax(pushing[::-1]) - 1)
    rest = ee[max(first - 1, 0)]

    predicted = 1000 * args.force / args.stiffness
    during = np.linalg.norm(ee[first:last + 1] - rest, axis=1) * 1000
    after = ee[last + 1:]

    print("--- under the push " + "-" * 34)
    # Both numbers, because an undamped spring is still swinging when the push
    # ends and the last sample of the window is then wherever the swing happened
    # to be -- which says nothing about its stiffness.
    print(f"deflection at release: {during[-1]:.1f} mm   "
          f"({during[-1] / predicted:.2f}x the F/K prediction)")
    print(f"peak during the push : {during.max():.1f} mm")
    print("--- after it is released " + "-" * 28)
    if len(after) > 10:
        offset = np.linalg.norm(after - rest, axis=1) * 1000
        outside = np.flatnonzero(offset > 1.0)
        settled = (outside[-1] + 1) * dt if outside.size else 0.0
        never = outside.size and outside[-1] == len(offset) - 1
        # A ring is the arm turning round and going back. Counted only while it
        # is still meaningfully displaced, because at a tenth of a millimetre
        # every numerical wobble is a direction reversal and none of them are
        # ringing.
        travel = after @ PUSH_DIRECTION
        big = offset[1:] > max(0.1 * during.max(), 1.0)
        vel = np.diff(travel)[big[:len(np.diff(travel))]]
        reversals = int(np.count_nonzero(np.diff(np.sign(vel)) != 0)) if vel.size else 0
        print(f"settling             : "
              + ("never came back inside 1 mm" if never else f"{settled:.2f} s to within 1 mm"))
        print(f"direction reversals  : {reversals}"
              + ("  — it is ringing" if reversals > 2 else ""))
        print(f"residual offset      : {offset[-1]:.2f} mm")


if __name__ == "__main__":
    raise SystemExit(main())
