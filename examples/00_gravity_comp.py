"""Milestone 0 — the arm holds itself up, and you can see what is holding it.

    python examples/00_gravity_comp.py              # viewer, compensation on
    python examples/00_gravity_comp.py --break-it   # compensation off: it falls
    python examples/00_gravity_comp.py --headless   # no window, prints the droop

What you are looking at: one arrow per arm joint, drawn along that joint's own
rotation axis, its length proportional to torque and its direction given by the
sign — the right-hand rule made visible.

    solid orange   g(q), what gravity asks of that joint. This is a property of
                   the pose, so compensating does NOT shrink it.
    faint yellow   what gravity still gets: g(q) minus what we sent back. This
                   is the one that vanishes when compensation is on, and the
                   one that reappears, at full length, under --break-it.
"""

import argparse
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from mujoManipulation.controller.gravity_comp import GravityCompensation
from mujoManipulation.env import MuJoCoEnv
from mujoManipulation.robot.panda import FrankaPanda

XML = os.path.join(
    "mujoManipulation", "assets", "panda", "franka_panda.xml"
)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--break-it",
        action="store_true",
        help="turn gravity compensation off and watch the arm sag",
    )
    parser.add_argument(
        "--headless", action="store_true", help="run without opening a window"
    )
    parser.add_argument(
        "--seconds", type=float, default=4.0, help="simulated seconds (headless)"
    )
    args = parser.parse_args()

    xml = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), XML)

    with MuJoCoEnv(xml, render=not args.headless) as env:
        panda = FrankaPanda(env.model, env.data)
        panda.go_home()

        controller = GravityCompensation(panda, enabled=not args.break_it)
        home = panda.HOME.copy()

        state = "OFF  (break it)" if args.break_it else "ON"
        load = controller.gravity_torque()
        print(f"gravity compensation : {state}")
        print(f"g(q) at home         : {np.round(load, 2)}  N·m")
        print(f"largest joint load   : {np.abs(load).max():.2f} N·m")

        n_steps = int(args.seconds / env.dt)
        started = time.time()
        worst_droop = 0.0

        for step in range(n_steps):
            if not env.is_running():
                break

            tau, primitives = controller.update()
            panda.set_arm_torque(tau)
            env.step()
            env.sync(primitives)

            worst_droop = max(
                worst_droop, float(np.abs(panda.arm_q - home).max())
            )

            if not args.headless:
                # keep the viewer roughly real time so it is watchable
                ahead = (step + 1) * env.dt - (time.time() - started)
                if ahead > 0:
                    time.sleep(ahead)

        print(f"worst joint droop    : {np.degrees(worst_droop):.3f}°")
        print(f"final arm error      : "
              f"{np.degrees(np.abs(panda.arm_q - home)).round(3)} deg")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
