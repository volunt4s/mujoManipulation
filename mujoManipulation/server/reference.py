"""One goal pose, and the ways of being asked to reach it.

The rungs used to carry a reference each — gravity compensation held the home
pose, the PID chased a square wave, computed torque tracked a 1 Hz stroke — which
made every comparison between them an apples-to-oranges one. Here the goal and
the motion belong to the *simulation*, and the controller is the only thing that
changes: same pose, same profile, different law, and the difference you see is
the law.

Goals are joint-space, because every controller on the ladder so far is. The
end-effector frame shown in the viewer is forward kinematics *of* the goal, not
the goal itself — which is the honest way round until OSC arrives and a Cartesian
goal starts to mean something. A joint-space law would then need inverse
kinematics to chase one, and that is a later rung's problem.

A task is two named poses and a profile between them. Naming the start rather
than capturing wherever the arm happened to be makes the reference a function of
(start, goal, duration, t) and nothing else — so the same task gives the same
demand every time, which is what makes two controllers comparable. `reset` parks
the arm at the start pose for the same reason.

The three profiles, all from the start pose to the goal:

    step        q_d is the goal from the first instant. Nothing is fed forward,
                because there is nothing smooth to feed: the demand is a
                discontinuity and only feedback can answer it. This is where
                PID's overshoot and its settling time come from.
    smooth      a minimum-jerk polynomial, so q̇_d and q̈_d exist and are
                continuous. This is the profile that lets computed torque show
                what feedforward buys: the PD part only corrects the difference
                rather than generating the whole motion.
    oscillate   smooth, start to goal and back, forever. The one to leave
                running while switching rungs underneath it.

The polynomial is written out rather than imported, like every other piece of
mathematics the repo teaches.
"""

import numpy as np

# Joint-space goals, checked against the joint limits and against each other for
# a spread of end-effector positions. HOME is index 0 so "nothing to do" is the
# first thing offered.
GOALS = [
    {"id": "home", "label": "home", "q": None},   # filled from the robot
    {"id": "reach", "label": "reach out",
     "q": [0.6, 0.35, 0.0, 0.5, 0.0, -0.5, 1.0], "relative": True},
    {"id": "extend", "label": "extended",
     "q": [0.0, 0.25, 0.0, -1.1, 0.0, 1.35, 0.785]},
    {"id": "side", "label": "to the side",
     "q": [1.3, -0.3, 0.0, -1.9, 0.0, 1.6, 0.785]},
]

MOTIONS = [
    {"id": "step", "label": "step"},
    {"id": "smooth", "label": "smooth"},
    {"id": "oscillate", "label": "oscillate"},
]

DURATION_RANGE = (0.5, 6.0)


def goal_pose(goal_id: str, home: np.ndarray) -> np.ndarray:
    for goal in GOALS:
        if goal["id"] != goal_id:
            continue
        if goal["q"] is None:
            return home.copy()
        q = np.asarray(goal["q"], dtype=float)
        return home + q if goal.get("relative") else q
    return home.copy()


def _minimum_jerk(tau: float):
    """s, ṡ, s̈ of the quintic that leaves and arrives at rest.

        s(τ)  = 10τ³ − 15τ⁴ + 6τ⁵
        s'(τ) = 30τ² − 60τ³ + 30τ⁴
        s"(τ) = 60τ  − 180τ² + 120τ³

    Both ends have zero velocity and zero acceleration, which is what keeps q̈_d
    finite and therefore worth feeding forward.
    """
    if tau <= 0.0:
        return 0.0, 0.0, 0.0
    if tau >= 1.0:
        return 1.0, 0.0, 0.0
    t2, t3, t4 = tau * tau, tau**3, tau**4
    return (
        10 * t3 - 15 * t4 + 6 * t4 * tau,
        30 * t2 - 60 * t3 + 30 * t4,
        60 * tau - 180 * t2 + 120 * t3,
    )


class Reference:
    """Where the arm is being asked to go, and how.

    `restart` captures the pose to leave from, so a profile always begins at the
    arm rather than snapping it somewhere first.
    """

    def __init__(self, home: np.ndarray):
        self.home = home.copy()
        self.start_id = "home"
        self.goal_id = "reach"
        # Oscillate by default because this is a viewer: a one-shot arrival is
        # the better measurement and a worse thing to open to, since the arm
        # finishes moving before anyone has looked at it. One second, because
        # the cost of dropping M(q) grows with q̈ — it is 9x the tracking error
        # at three seconds and 20x at half a second.
        self.motion = "oscillate"
        self.duration = 1.0
        self.start_t = 0.0
        self.start_q = goal_pose(self.start_id, home)
        self.goal = goal_pose(self.goal_id, home)

    def restart(self, t_now: float) -> None:
        """Put the profile's clock back to `t_now`. The poses do not move."""
        self.start_t = float(t_now)

    def set_start(self, start_id: str, t_now) -> None:
        if start_id in {g["id"] for g in GOALS}:
            self.start_id = start_id
            self.start_q = goal_pose(start_id, self.home)
            self.restart(t_now)

    def set_goal(self, goal_id: str, t_now) -> None:
        if goal_id in {g["id"] for g in GOALS}:
            self.goal_id = goal_id
            self.goal = goal_pose(goal_id, self.home)
            self.restart(t_now)

    def set_motion(self, motion: str, t_now) -> None:
        if motion in {m["id"] for m in MOTIONS}:
            self.motion = motion
            self.restart(t_now)

    def set_duration(self, seconds: float, t_now) -> None:
        self.duration = float(np.clip(seconds, *DURATION_RANGE))
        self.restart(t_now)

    def sample(self, t: float):
        """q_d, q̇_d, q̈_d at time `t`."""
        span = self.goal - self.start_q
        zero = np.zeros_like(span)

        if self.motion == "step":
            return self.goal.copy(), zero, zero

        elapsed = max(t - self.start_t, 0.0)

        if self.motion == "oscillate":
            # Out and back on the same minimum-jerk shape, so the turnaround has
            # no corner in it either.
            phase = (elapsed / self.duration) % 2.0
            tau = phase if phase <= 1.0 else 2.0 - phase
            direction = 1.0 if phase <= 1.0 else -1.0
            s, sd, sdd = _minimum_jerk(tau)
            return (
                self.start_q + span * s,
                span * sd * direction / self.duration,
                span * sdd / self.duration**2,
            )

        s, sd, sdd = _minimum_jerk(elapsed / self.duration)
        return (
            self.start_q + span * s,
            span * sd / self.duration,
            span * sdd / self.duration**2,
        )

    def state(self) -> dict:
        return {"start": self.start_id, "goal": self.goal_id,
                "motion": self.motion, "duration": round(self.duration, 2)}
