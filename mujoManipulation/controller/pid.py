"""Joint-space PID — the first feedback rung of the ladder.

Gravity compensation (rung 0) is feedforward: it asks the model what gravity is
doing and sends exactly that back. It has no idea where the arm actually is. The
moment you want the arm to *go somewhere*, you need the other half — a law that
looks at the gap between where it is and where you asked it to be, and pushes.

That is this file. Per joint, with e = q_d − q and ė = q̇_d − q̇:

    τ  =  K_p e  +  K_i ∫ e dt  +  K_d ė

Three terms, three jobs, and each one is visible in the demo as its own arrow:

    K_p e       the spring. Proportional to how wrong you are right now. Alone,
                it stalls where its pull balances the load — against gravity
                that is a permanent droop of g(q)/K_p radians, not zero.
    K_i ∫e dt   the memory. It keeps adding while the error persists, so a
                constant load that P cannot beat is eventually cancelled by an
                integral that grew until it could. This is how PID defeats
                gravity *without being told gravity exists*.
    K_d ė       the damper. Proportional to how fast the error is closing, so it
                opposes the approach and stops P from sailing past the target.
                Remove it and the arm rings like a struck bell — the joints are
                barely damped in the XML (0.1 N·m·s/rad), so nothing else is
                there to absorb the energy P keeps pumping in.

Nothing about this law knows what it is driving. It does not know the arm's
mass, that gravity exists, or that the joints are coupled — it sees a number
that should be zero and pushes against it. That ignorance is the whole appeal
and also the ceiling: every rung above this one buys its accuracy by putting
some of the model back in.

Two details that are not in the textbook equation but are in every working
implementation, and are the two things this law carried over from the
pre-rewrite code:

anti-windup
    An actuator has a ctrlrange. Once it is pinned at ±80 N·m, extra integral
    buys nothing but still accumulates — and then has to be unwound before the
    joint can turn around, so the arm sits there, pointed the wrong way, for as
    long as it takes. So the integral is only advanced on channels whose
    unclipped demand still fits inside the limits. Saturated channel: the memory
    holds, it does not grow. Measured on the demo's unreachable target, where
    joint 4 presses into its own limit for 3.5 s: the integral reaches −306 N·m
    on an ±80 N·m channel without this, and −35 N·m with it.

filtered derivative
    ė is passed through a one-pole lowpass with time constant `d_filter_tau`
    before it reaches the output. In simulation q̇ is exact and the filter
    changes almost nothing; on hardware q̇ is differenced from an encoder, which
    divides by dt and so multiplies every count of quantisation noise by
    1/dt = 500, and the filter is the only reason the D term is usable at all.
    It is here because the law should be the one you could actually run.

One line changed from the pre-rewrite version, which took ė from differencing
the error, (e − e_prev)/dt, instead of from the measured q̇. While the reference
holds still the two agree exactly — a constant q_d makes (e − e_prev)/dt equal
to −q̇ — so the difference lives entirely at the moment the reference *moves*,
where it is one sample of Δq_d/dt = 500·Δq_d. Measured on the demo's square
wave, at the instant the reference flips: 1372 N·m demanded of joint 1's 80 N·m
actuator (7872 N·m with the lowpass switched off, so the filter is worth five
sixths of the spike and still cannot save it), against 72 N·m when ė comes from
q̇. Clipping means it does not break anything — it just pins the actuator at its
limit for the first few steps after every reference change, for no reason.
Taking ė from q̇ costs nothing, because q̇ is a state the plant already has. The
anti-windup and the lowpass carried over; this one line did not.

On dt: this is the interval the control loop actually runs at, and it belongs to
the caller's loop, not to this class — pass `env.dt`. Both the I and the D term
scale by it, so a dt that disagrees with the real step size silently retunes two
thirds of the controller. (The pre-rewrite code was constructed with dt=0.001
against a 0.002 timestep, which is exactly this bug.)
"""

import mujoco
import numpy as np

from ..viz.primitives import Arrow, GhostPath

# Same scale as the gravity-comp demo, so an arrow here is directly comparable
# to an arrow there: one metre of arrow is this many newton-metres.
NM_PER_METRE = 40.0

# Lowpass time constant for the derivative, in seconds. Must sit above dt (or
# it filters nothing) and below the arm's own response time (or it adds lag to
# the one term whose job is to lead).
D_FILTER_TAU = 0.01

# Public because the web layer sends these to the browser: a term's colour in
# the equation panel has to be the same colour as its arrow, and this is the
# only place that decides it.
P_RGBA = (0.25, 0.5, 0.95, 0.95)     # the spring
I_RGBA = (0.35, 0.8, 0.4, 0.85)      # the memory
D_RGBA = (0.95, 0.6, 0.15, 0.55)     # the damper
GHOST_RGBA = (0.75, 0.75, 0.8, 0.45)  # where you asked it to be


class JointSpacePID:
    """PID on the 7 arm joints. Torque in, nothing else.

    Arm only, deliberately. The gripper is two *position* actuators in metres;
    feeding them a number sized for a newton-metre channel is what the old
    all-`nu` version did, and the fingers twitching was the symptom.
    """

    def __init__(
        self,
        robot,
        K_p,
        K_i,
        K_d,
        dt: float,
        d_filter_tau: float = D_FILTER_TAU,
        d_enabled: bool = True,
        i_enabled: bool = True,
        anti_windup: bool = True,
        filter_derivative: bool = True,
        nm_per_metre: float = NM_PER_METRE,
    ):
        self.robot = robot
        n = robot.n_arm

        # Gains are per-joint. A scalar broadcasts, which is fine for a toy,
        # but joint 1 carries the whole arm and joint 7 carries a gripper —
        # they do not want the same stiffness.
        self.K_p = self._gains(K_p, n, "K_p")
        self.K_i = self._gains(K_i, n, "K_i")
        self.K_d = self._gains(K_d, n, "K_d")

        self.dt = float(dt)
        self.d_filter_tau = float(d_filter_tau)

        self.d_enabled = d_enabled
        self.i_enabled = i_enabled
        self.anti_windup = anti_windup
        self.filter_derivative = filter_derivative
        self.nm_per_metre = nm_per_metre

        self.torque_limits = np.asarray(robot.arm_torque_limits, dtype=float)

        # Scratch state, used only to run forward kinematics on the *desired*
        # pose so the demo can draw it. Never stepped.
        self._scratch = mujoco.MjData(robot.model)

        self.reset()

    @staticmethod
    def _gains(gain, n: int, name: str) -> np.ndarray:
        gain = np.asarray(gain, dtype=float)
        if gain.ndim == 0:
            return np.full(n, float(gain))
        if gain.shape != (n,):
            raise ValueError(f"{name} must be a scalar or {n} values, got {gain.shape}")
        return gain.copy()

    def reset(self) -> None:
        """Forget the integral and the derivative history."""
        n = self.robot.n_arm
        self.integral = np.zeros(n)  # holds K_i ∫e dt, gain already folded in
        self.error = np.zeros(n)
        self.error_rate = np.zeros(n)
        self.p_term = np.zeros(n)
        self.i_term = np.zeros(n)
        self.d_term = np.zeros(n)
        self.saturated = np.zeros(n, dtype=bool)
        self._d_prev = np.zeros(n)

    # --- the law ----------------------------------------------------------

    def update(self, q_desired, qd_desired=None):
        """Returns (torque, draw primitives). The torque is 7 arm channels.

            e = q_d − q            ė = q̇_d − q̇
            τ = K_p e  +  K_i ∫ e dt  +  K_d ė

        `qd_desired` defaults to zero, which is what you want for a pose you are
        asked to hold or step to. Pass it when the reference is a trajectory:
        without it the D term reads the arm's own tracking velocity as error
        closing speed and brakes against motion you asked for.
        """
        q_desired = np.asarray(q_desired, dtype=float)
        e = q_desired - self.robot.arm_q
        ed = (0.0 if qd_desired is None else np.asarray(qd_desired, dtype=float)) \
            - self.robot.arm_qd

        p_term = self.K_p * e
        i_term = self.integral if self.i_enabled else np.zeros_like(e)
        d_term = self._derivative(ed)

        tau_demanded = p_term + i_term + d_term
        tau = np.clip(tau_demanded, self.torque_limits[:, 0], self.torque_limits[:, 1])

        # Anti-windup: advance the memory only where the demand still fits. A
        # channel that is already pinned cannot act on more integral, so letting
        # it grow only buys an overshoot later.
        self.saturated = tau_demanded != tau
        growing = ~self.saturated if self.anti_windup else np.ones_like(e, dtype=bool)
        self.integral += self.K_i * e * self.dt * growing

        self.error, self.error_rate = e, ed
        self.p_term, self.i_term, self.d_term = p_term, i_term, d_term

        return tau, self._draw(q_desired)

    def _derivative(self, ed: np.ndarray) -> np.ndarray:
        """K_d ė, lowpassed.

            d ← α d_prev + (1 − α) K_d ė,     α = τ_f / (τ_f + dt)

        α is how much of the previous output survives: at τ_f = 10 ms and
        dt = 2 ms it is 0.83, so a spike needs a few steps to get through and a
        sustained value still arrives in full.
        """
        if not self.d_enabled:
            self._d_prev = np.zeros_like(ed)
            return self._d_prev

        raw = self.K_d * ed

        if self.filter_derivative:
            alpha = self.d_filter_tau / (self.d_filter_tau + self.dt)
            raw = alpha * self._d_prev + (1.0 - alpha) * raw

        self._d_prev = raw
        return raw

    # --- drawing ----------------------------------------------------------

    def _draw(self, q_desired: np.ndarray) -> list:
        """The three terms, and the pose they are arguing about.

        Each term gets an arrow at the joint it acts on, along that joint's own
        rotation axis, length proportional to torque and direction given by the
        sign — the same convention as the gravity-comp demo, so the two are
        readable side by side. Widths differ so the three stay distinguishable
        where they overlap:

            thin, blue      K_p e
            medium, green   K_i ∫e dt
            wide, faint     K_d ė

        and the pale polyline is the arm the reference describes: the skeleton
        the real one is chasing. The gap between them *is* e.
        """
        primitives = [
            GhostPath(points=self._desired_skeleton(q_desired), width=0.012,
                      rgba=GHOST_RGBA)
        ]

        anchors = self.robot.arm_joint_anchors
        axes = self.robot.arm_joint_axes

        for anchor, axis, d_i, i_i, p_i in zip(
            anchors, axes, self.d_term, self.i_term, self.p_term
        ):
            for value, width, rgba in (
                (d_i, 0.022, D_RGBA),
                (i_i, 0.014, I_RGBA),
                (p_i, 0.008, P_RGBA),
            ):
                primitives.append(
                    Arrow(
                        origin=anchor,
                        vec=axis * (value / self.nm_per_metre),
                        width=width,
                        rgba=rgba,
                    )
                )
        return primitives

    def _desired_skeleton(self, q_desired: np.ndarray) -> np.ndarray:
        """Joint anchors of the *desired* pose, as a polyline from base to wrist.

        Forward kinematics on a throwaway MjData: ask the model where the arm
        would be if it were already there. This is the model used as a model,
        not as a controller — the law above never touches it.
        """
        scratch = self._scratch
        scratch.qpos[:] = self.robot.data.qpos
        scratch.qpos[self.robot.arm_qpos_adr] = q_desired
        mujoco.mj_kinematics(self.robot.model, scratch)
        return scratch.xanchor[self.robot.arm_joint_ids].copy()
