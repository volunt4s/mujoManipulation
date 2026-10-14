"""The fixed-rate simulation loop, on its own thread.

The library's rule is that the loop lives in the caller (see `examples/`), and
this file is simply another caller — a long-lived one that happens to be driven
by a socket instead of by `__main__`. Nothing in `env / robot / controller /
viz` knows this file exists, and switching the server off leaves every example
working exactly as before.

Two rates, which is the only subtle thing in here:

    the physics   dt = 0.002 s, so 500 steps a second, because that is the
                  timestep the controllers were tuned against and changing it
                  would silently retune them
    the stream    60 frames a second, because that is what a screen does

So one frame in eight is published. The sim thread builds the published frame
itself, under a lock, and the socket handler only ever reads the latest one —
a slow or stalled client therefore drops frames instead of dragging the physics
down with it, and a fast one cannot ask for frames that do not exist yet.

Pacing is done by *catch-up*, not by sleeping a timestep at a time: `sleep(dt)`
with dt = 2 ms is at the mercy of the OS scheduler and drifts. Instead the loop
asks how many steps are due by now, runs those, and sleeps a millisecond. If it
ever falls far behind — a laptop lid, a breakpoint — it gives up on the lost
time rather than sprinting through it, because a 3-second fast-forward is worse
than a 3-second gap.
"""

import threading
import time

import numpy as np

from ..controller.computed_torque import ComputedTorque
from ..env import MuJoCoEnv
from ..robot.panda import FrankaPanda
from .protocol import state_message
from .scene import Scene

# W1 runs one hardcoded scenario: the stroke from examples/02, because an arm
# that holds still proves nothing about a streaming pipeline. W3 replaces this
# with the controller picker, at which point these constants move with it.
STROKE_AMPLITUDE = np.array([0.9, 0.5, 0.0, 0.7, 0.6, 0.5, 0.0])
STROKE_HZ = 1.0
OMEGA, ZETA = 20.0, 1.0

# Never run more than this many physics steps in one wake-up.
MAX_CATCHUP_STEPS = 50


class SimulationThread(threading.Thread):
    def __init__(self, xml_path: str, stream_hz: float = 60.0):
        super().__init__(name="sim", daemon=True)

        self.env = MuJoCoEnv(xml_path)          # headless, always
        self.panda = FrankaPanda(self.env.model, self.env.data)
        self.panda.go_home()
        self.scene = Scene(self.env.model, xml_path)

        self.controller = ComputedTorque(self.panda, K_p=OMEGA**2, K_d=2 * ZETA * OMEGA)
        self.home = self.panda.HOME.copy()

        self.dt = self.env.dt
        self.stream_hz = float(stream_hz)
        self.publish_every = max(1, round((1.0 / self.dt) / self.stream_hz))

        self.speed = 1.0
        self._steps = 0
        self._frame: dict | None = None
        self._lock = threading.Lock()
        self._stop = threading.Event()

    # --- the reference ----------------------------------------------------

    def reference(self, t: float):
        """q_d, q̇_d, q̈_d — the same (1 − cos) stroke examples/02 tracks."""
        omega = 2.0 * np.pi * STROKE_HZ
        half = STROKE_AMPLITUDE / 2.0
        return (
            self.home + half * (1.0 - np.cos(omega * t)),
            half * omega * np.sin(omega * t),
            half * omega**2 * np.cos(omega * t),
        )

    # --- the loop ---------------------------------------------------------

    def run(self) -> None:
        self._publish()                      # frame 0, so a client that
        started = time.perf_counter()        # connects first sees something

        while not self._stop.is_set():
            elapsed = (time.perf_counter() - started) * self.speed
            due = int(elapsed / self.dt) - self._steps

            if due > MAX_CATCHUP_STEPS:
                # Too far behind to make up honestly. Forget the lost time.
                self._steps = int(elapsed / self.dt)
                due = 1

            for _ in range(max(due, 0)):
                self._step_once()

            time.sleep(0.001)

    def _step_once(self) -> None:
        t = self.env.data.time
        q_d, qd_d, qdd_d = self.reference(t)

        tau, primitives = self.controller.update(q_d, qd_d, qdd_d)
        self.panda.set_arm_torque(tau)
        self.env.step()
        self._steps += 1

        if self._steps % self.publish_every == 0:
            self._publish(q_d, tau, primitives)

    def _publish(self, q_d=None, tau=None, primitives=()) -> None:
        """Build the frame the socket will send. Called from the sim thread."""
        if q_d is None:
            q_d, _, _ = self.reference(self.env.data.time)
            tau = np.zeros(self.panda.n_arm)

        error = np.degrees(self.panda.arm_q - q_d)
        frame = state_message(
            self.env.data.time,
            self.scene.transforms(self.env.data),
            primitives,
            channels={
                "error_deg": [round(v, 4) for v in error.tolist()],
                "torque_nm": [round(v, 3) for v in np.asarray(tau).tolist()],
            },
        )
        with self._lock:
            self._frame = frame

    # --- read side --------------------------------------------------------

    def frame(self) -> dict | None:
        """The most recent published frame. Safe to call from any thread."""
        with self._lock:
            return self._frame

    def stop(self) -> None:
        self._stop.set()
