"""The fixed-rate simulation loop, on its own thread.

The library's rule is that the loop lives in the caller (see `examples/`), and
this file is simply another caller — a long-lived one driven by a socket
instead of by `__main__`. Nothing in `env / robot / controller / viz` knows this
file exists, and switching the server off leaves every example working exactly
as before.

Two rates, which is the only subtle thing in here:

    the physics   dt = 0.002 s, so 500 steps a second, because that is the
                  timestep the controllers were tuned against and changing it
                  would silently retune them
    the stream    60 frames a second, because that is what a screen does

So one frame in eight is published. The sim thread builds the published frame
itself, under a lock, and the socket handler only ever reads the latest one —
a slow or stalled client therefore drops frames instead of dragging the physics
down with it.

Pacing is by *budget* rather than by sleeping a timestep at a time: `sleep(dt)`
with dt = 2 ms is at the mercy of the scheduler and drifts, and an absolute
`steps == elapsed/dt` schedule jumps the moment anyone touches the speed
multiplier. So each wake-up adds the wall time since the last one — scaled by
`speed`, and skipped entirely while paused — to a budget of owed simulation
seconds, and spends whatever whole steps it can afford. Falling far behind, from
a closed laptop lid or a breakpoint, forfeits the lost time instead of sprinting
through it: a 3-second fast-forward is worse than a 3-second gap.

Commands from the browser are queued and applied *between* steps, never during
one, so a controller is never swapped out from under a half-finished update.
"""

import queue
import threading
import time

import mujoco
import numpy as np

from ..env import MuJoCoEnv
from ..robot.panda import FrankaPanda
from ..viz.primitives import Arrow, Frame
from .model import load_model
from .protocol import state_message
from .reference import DURATION_RANGE, Reference
from .scene import Scene
from .scenarios import DEFAULT, SCENARIOS, valid_off

# Never run more than this many physics steps in one wake-up.
MAX_CATCHUP_STEPS = 50

# What the arrow-scale control may be set to, in newton-metres per metre.
SCALE_RANGE = (10.0, 200.0)
SPEED_RANGE = (0.05, 4.0)

# The two end-effector frames the viewer can draw. The goal's is the larger and
# heavier of the two, because `Frame` carries no colour — x red, y green, z blue
# is the convention and overriding it to tell two frames apart would cost more
# than it bought.
EE_FRAME = {"length": 0.10, "width": 0.005}
GOAL_FRAME = {"length": 0.15, "width": 0.011}

# A shove at the gripper, for the rung whose subject is what happens when the
# world disagrees. One second is long enough to settle under and short enough to
# watch the recovery.
PUSH_SECONDS = 1.0
PUSH_RANGE = (0.0, 80.0)
PUSH_AXES = {"x": (1.0, 0.0, 0.0), "y": (0.0, 1.0, 0.0), "z": (0.0, 0.0, 1.0)}
PUSH_RGBA = (0.95, 0.25, 0.3, 0.95)
N_PER_METRE = 80.0


class SimulationThread(threading.Thread):
    def __init__(self, xml_path: str, stream_hz: float = 60.0, use_cache: bool = True):
        super().__init__(name="sim", daemon=True)

        # Headless, always, and from the compiled cache when there is one: a
        # server that will eventually hold a simulation per connection cannot
        # afford to compile the XML per connection.
        self.env = MuJoCoEnv(xml_path, model=load_model(xml_path, use_cache))
        self.panda = FrankaPanda(self.env.model, self.env.data)
        self.scene = Scene(self.env.model, xml_path)

        self.dt = self.env.dt
        self.stream_hz = float(stream_hz)
        self.publish_every = max(1, round((1.0 / self.dt) / self.stream_hz))

        self.speed = 1.0
        self.paused = False
        self.scale = 40.0
        self.show = {"ee": True, "goal": True}
        self.push_newtons = 20.0
        self.push_force = np.zeros(3)
        self.push_until = -1.0
        self.ee_body = int(self.env.model.site_bodyid[self.panda.ee_site_id])

        # Where the arm is going, and how — owned here rather than by the
        # controller, so that swapping the controller changes the law and not
        # the task.
        self.panda.go_home()
        self.reference = Reference(self.panda.HOME)

        # Scratch state for one job: forward kinematics of the *goal* pose, so
        # the viewer can show the frame the arm is being asked to reach.
        self._scratch = mujoco.MjData(self.env.model)

        self._commands: queue.Queue = queue.Queue()
        self._steps = 0
        self._budget = 0.0
        self._frame: dict | None = None
        self._lock = threading.Lock()
        self._stop = threading.Event()

        self.run_id = None
        self.off = frozenset()
        self.select(DEFAULT)

    # --- what is running --------------------------------------------------

    def select(self, run_id: str, off=()) -> None:
        """Swap the controller, in place. Called on the sim thread.

        Nothing else moves: not the arm, not the goal, not the clock. Switching
        the law mid-motion and watching the arrows change is the comparison this
        whole thing exists for, and re-homing first would throw it away. `reset`
        is there for when you do want a clean start.
        """
        if run_id not in SCENARIOS:
            return
        self.run_id = run_id
        self.off = valid_off(run_id, off)
        self.scenario = SCENARIOS[run_id](self.panda, self.dt, self.off)
        self.scenario.set_scale(self.scale)
        self._publish()

    def restart(self) -> None:
        """Clock to zero, arm parked at the task's start pose, controller fresh.

        The start pose rather than the home pose, because the task is defined as
        two named poses: putting the arm anywhere else would make the first
        instant of every profile a step the task never asked for.
        """
        self.env.reset()
        self.push_until = -1.0
        self.env.data.xfrc_applied[:] = 0.0
        self.panda.go_home()
        self.env.data.qpos[self.panda.arm_qpos_adr] = self.reference.start_q
        self.env.data.qvel[self.panda.arm_dof_adr] = 0.0
        mujoco.mj_forward(self.env.model, self.env.data)
        self._steps = 0
        self._budget = 0.0
        self.reference.restart(self.env.data.time)
        # `reset`, not a rebuild. Rebuilding would hand the scenario its class
        # defaults back and silently undo every gain the slider had moved --
        # restarting a run is meant to restart the motion, not the tuning.
        # The controllers' own reset() is what clears an integral or a filter.
        self.scenario.reset()
        self._publish()

    def state(self) -> dict:
        """The switch positions, so a client can render its own controls."""
        return {
            "scenario": self.run_id,
            "off": sorted(self.off),
            "gains": self.scenario.gain_values(),
            "scale": self.scale,
            "speed": self.speed,
            "paused": self.paused,
            "show": dict(self.show),
            "push_newtons": round(self.push_newtons, 1),
            "pushing": bool(self.env.data.time < self.push_until),
            **self.reference.state(),
        }

    # --- commands ---------------------------------------------------------

    def submit(self, command: dict) -> None:
        """Queue a command from any thread. Applied before the next step."""
        self._commands.put(command)

    def _drain(self) -> None:
        touched = False
        while True:
            try:
                command = self._commands.get_nowait()
            except queue.Empty:
                break
            self._apply(command)
            touched = True
        if touched and self.paused:
            # Nothing will step, so nothing would otherwise redraw.
            self._publish()

    def _apply(self, command: dict) -> None:
        op = command.get("op")
        if op == "select":
            self.select(command.get("scenario", self.run_id), command.get("off", ()))
        elif op == "term":
            # Flipped on the live controller rather than by rebuilding it: a
            # term switched off mid-motion is the whole point, and a rebuild
            # would quietly reset whatever state the law was carrying.
            term = command.get("id")
            if term in {sw["id"] for sw in SCENARIOS[self.run_id].switches}:
                on = bool(command.get("on", term in self.off))
                self.off = (self.off - {term}) if on else (self.off | {term})
                self.scenario.set_switch(term, on)
        elif op == "gain":
            self.scenario.set_gain(command.get("id"), command.get("value", 0.0))
        elif op == "start":
            self.reference.set_start(command.get("id"), self.env.data.time)
        elif op == "goal":
            self.reference.set_goal(command.get("id"), self.env.data.time)
        elif op == "motion":
            self.reference.set_motion(command.get("id"), self.env.data.time)
        elif op == "duration":
            self.reference.set_duration(
                command.get("value", self.reference.duration), self.env.data.time)
        elif op == "push":
            axis = PUSH_AXES.get(command.get("axis"))
            if axis is not None:
                if "value" in command:
                    self.push_newtons = float(
                        np.clip(command["value"], *PUSH_RANGE))
                sign = -1.0 if command.get("negative") else 1.0
                self.push_force = sign * self.push_newtons * np.array(axis)
                self.push_until = self.env.data.time + PUSH_SECONDS
            elif "value" in command:
                self.push_newtons = float(np.clip(command["value"], *PUSH_RANGE))
        elif op == "show":
            key = command.get("id")
            if key in self.show:
                self.show[key] = bool(command.get("value", not self.show[key]))
        elif op == "scale":
            self.scale = float(np.clip(command.get("value", self.scale), *SCALE_RANGE))
            self.scenario.set_scale(self.scale)
        elif op == "speed":
            self.speed = float(np.clip(command.get("value", self.speed), *SPEED_RANGE))
        elif op == "pause":
            self.paused = bool(command.get("value", not self.paused))
        elif op == "reset":
            self.restart()
        elif op == "step":
            for _ in range(int(command.get("n", 1))):
                self._step_once()

    # --- the loop ---------------------------------------------------------

    def run(self) -> None:
        last = time.perf_counter()

        while not self._stop.is_set():
            self._drain()

            now = time.perf_counter()
            if not self.paused:
                self._budget += (now - last) * self.speed
            last = now

            due = int(self._budget / self.dt)
            if due > MAX_CATCHUP_STEPS:
                due, self._budget = 1, 0.0      # forfeit the lost time
            else:
                self._budget -= due * self.dt

            for _ in range(max(due, 0)):
                self._step_once()

            time.sleep(0.001)

    def _step_once(self) -> None:
        # The push is the one force in here that is not the controller's. It is
        # applied to the plant directly, exactly as a hand would be, so that the
        # controller finds out about it the only way a real one can: by being
        # moved.
        pushing = self.env.data.time < self.push_until
        self.env.data.xfrc_applied[self.ee_body] = 0.0
        if pushing:
            self.env.data.xfrc_applied[self.ee_body, :3] = self.push_force

        q_d, qd_d, qdd_d = self.reference.sample(self.env.data.time)
        tau, primitives = self.scenario.step(q_d, qd_d, qdd_d)
        self.panda.set_arm_torque(tau)
        self.env.step()
        self._steps += 1

        if self._steps % self.publish_every == 0:
            self._publish(q_d, tau, primitives)

    def _publish(self, q_ref=None, tau=None, primitives=()) -> None:
        """Build the frame the socket will send. Called from the sim thread."""
        if q_ref is None:
            q_ref = self.reference.sample(self.env.data.time)[0]
            tau = np.zeros(self.panda.n_arm)

        frame = state_message(
            self.env.data.time,
            self.scene.transforms(self.env.data),
            list(primitives) + self._annotations(q_ref),
            channels={
                "error_deg": [round(v, 4) for v in
                              np.degrees(self.panda.arm_q - q_ref).tolist()],
                "torque_nm": [round(v, 3) for v in np.asarray(tau).tolist()],
            },
        )
        frame["active"] = self.state()
        with self._lock:
            self._frame = frame

    def _annotations(self, q_ref) -> list:
        """Draw primitives that belong to the scene rather than to the law.

        The controllers draw the terms of their own equation; where the end
        effector is and where it has been asked to be are facts about the task,
        true of every rung, so they are added here instead.
        """
        drawn = []
        if self.show["ee"]:
            drawn.append(Frame(pos=self.panda.ee_pos, mat=self.panda.ee_mat,
                               **EE_FRAME))
        if self.show["goal"]:
            pos, mat = self._goal_ee_pose(q_ref)
            drawn.append(Frame(pos=pos, mat=mat, **GOAL_FRAME))
        if self.env.data.time < self.push_until:
            drawn.append(Arrow(origin=self.panda.ee_pos,
                               vec=self.push_force / N_PER_METRE,
                               width=0.015, rgba=PUSH_RGBA))
        return drawn

    def _goal_ee_pose(self, q_ref):
        """Forward kinematics of the goal, on a throwaway MjData.

        The goal is a joint-space pose, so its end-effector frame has to be
        asked for — this is the model used as a model, which is the one use of
        it the repo takes for granted.
        """
        scratch = self._scratch
        scratch.qpos[:] = self.env.data.qpos
        scratch.qpos[self.panda.arm_qpos_adr] = q_ref
        mujoco.mj_kinematics(self.env.model, scratch)
        mujoco.mj_comPos(self.env.model, scratch)   # sites need this pass too
        site = self.panda.ee_site_id
        return (scratch.site_xpos[site].copy(),
                scratch.site_xmat[site].reshape(3, 3).copy())

    # --- read side --------------------------------------------------------

    def frame(self) -> dict | None:
        """The most recent published frame. Safe to call from any thread."""
        with self._lock:
            return self._frame

    def stop(self) -> None:
        self._stop.set()
