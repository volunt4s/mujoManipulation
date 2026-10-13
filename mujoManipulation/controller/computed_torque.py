"""Computed torque — the rung where the model comes back.

PID pushes against a number that should be zero and knows nothing else. It
works, and the price is that every gain is a local fact: K_p that behaves at the
home pose is too soft with the arm stretched out, because the inertia the joint
has to shift changed underneath it. Seven joints, seven hand-tuned pairs, all of
them valid in one neighbourhood.

Computed torque buys its way out of that by putting the model back in. Start
from the manipulator equation, the whole thing this time:

    M(q) q̈  +  C(q, q̇) q̇  +  g(q)  =  τ

Solve it backwards. Decide the acceleration you *want*, and ask what torque
produces exactly that:

    q̈_cmd  =  q̈_d  +  K_d ė  +  K_p e            e = q_d − q,  ė = q̇_d − q̇

    τ  =  M(q) q̈_cmd  +  h(q, q̇)                 h = C(q, q̇) q̇ + g(q)

Substitute the second line back into the manipulator equation and h cancels, M
cancels, and what is left is

    ë  +  K_d ė  +  K_p e  =  0

a linear, decoupled, second-order error per joint — with no q in it. That is the
whole point and it is worth staring at: the arm's mass, its pose, the way joint 4
swinging throws joint 2 around, all of it has been divided out. Every joint now
obeys the same textbook spring-damper, so the gains stop being local facts and
start being *specifications*:

    K_p = ω²        K_d = 2ζω

Pick how fast (ω, rad/s) and how damped (ζ) you want the error to die, and you
are done — one pair of numbers for the whole arm, where PID needed seven. This
is feedback linearisation: the nonlinear plant is cancelled, and a linear one is
put in its place by hand.

There is also no integral term here, and that is not an omission. PID needed I
to discover a load it was never told about; this law is told. If an offset does
survive, it is measuring the gap between the model and the plant, which is
information worth seeing rather than integrating away.

Where h comes from, and why it is split
    `data.qfrc_bias` is C(q, q̇) q̇ + g(q) already summed, which is all the law
    needs. But the two halves fail differently and the demo has to show that, so
    g(q) is evaluated separately on a scratch MjData with q̇ = 0 — the same trick
    the gravity-comp controller uses — and the Coriolis part is recovered as the
    remainder, C(q, q̇) q̇ = qfrc_bias − g(q). Both are then drawn separately, and
    each can be switched off on its own:

        no g(q)     the arm sags, and then sags *while tracking* — a constant
                    bias the PD has to fight at every pose
        no C q̇      nothing visible at all until the arm moves quickly, because
                    these terms scale with q̇². They are the price of a joint
                    being dragged through a frame that is itself rotating, and
                    at a tenth of the speed they are a hundredth of the problem

Why M(q) is taken from MuJoCo
    `mj_fullM` is the plant, not the algorithm — the same reason the gravity
    rung asks the model for g(q). What is written out here is the control law:
    the acceleration command, the substitution, and the choice to cancel.
"""

import mujoco
import numpy as np

from ..viz.primitives import Arrow, GhostPath

# Same scale as every other rung: one metre of arrow is this many newton-metres.
NM_PER_METRE = 40.0

_INERTIAL_RGBA = (0.45, 0.35, 0.95, 0.95)   # M(q) q̈_cmd
_CORIOLIS_RGBA = (0.15, 0.75, 0.8, 0.85)    # C(q, q̇) q̇
_GRAVITY_RGBA = (0.95, 0.35, 0.15, 0.55)    # g(q), the colour it had in rung 0
_GHOST_RGBA = (0.75, 0.75, 0.8, 0.45)       # where you asked it to be


class ComputedTorque:
    """Inverse-dynamics control on the 7 arm joints.

    `K_p` and `K_d` are in the units of the *error dynamics*, not of torque:
    ë + K_d ė + K_p e = 0. A scalar pair is the right thing to pass, because
    after the cancellation every joint is the same system.
    """

    def __init__(
        self,
        robot,
        K_p,
        K_d,
        use_inertia: bool = True,
        use_coriolis: bool = True,
        use_gravity: bool = True,
        nm_per_metre: float = NM_PER_METRE,
    ):
        self.robot = robot
        n = robot.n_arm

        self.K_p = self._gains(K_p, n, "K_p")
        self.K_d = self._gains(K_d, n, "K_d")

        self.use_inertia = use_inertia
        self.use_coriolis = use_coriolis
        self.use_gravity = use_gravity
        self.nm_per_metre = nm_per_metre

        self.torque_limits = np.asarray(robot.arm_torque_limits, dtype=float)

        # Workspace. `_M_full` is nv×nv because mj_fullM fills the whole thing;
        # the arm block is sliced out of it afterwards.
        self._M_full = np.zeros((robot.model.nv, robot.model.nv))
        self._scratch = mujoco.MjData(robot.model)
        self._scratch_bias = np.zeros(robot.model.nv)

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
        """Clear the recorded quantities. There is no controller state to lose —
        no integral, no filter — so this only affects what gets drawn."""
        n = self.robot.n_arm
        self.error = np.zeros(n)
        self.error_rate = np.zeros(n)
        self.qdd_cmd = np.zeros(n)
        self.inertia = np.zeros((n, n))  # M(q) as of the last update, for the demo
        self.tau_inertial = np.zeros(n)
        self.tau_coriolis = np.zeros(n)
        self.tau_gravity = np.zeros(n)
        self.saturated = np.zeros(n, dtype=bool)

    # --- the law ----------------------------------------------------------

    def update(self, q_desired, qd_desired=None, qdd_desired=None):
        """Returns (torque, draw primitives). The torque is 7 arm channels.

            q̈_cmd = q̈_d + K_d ė + K_p e
            τ     = M(q) q̈_cmd + C(q, q̇) q̇ + g(q)

        `qd_desired` and `qdd_desired` default to zero, which asks the arm to
        arrive and stop. Pass all three to track a trajectory — and note that
        q̈_d going in as a feedforward is most of why this rung tracks: the PD
        part is then only correcting the difference, not generating the motion.
        """
        n = self.robot.n_arm
        q_desired = np.asarray(q_desired, dtype=float)
        qd_desired = np.zeros(n) if qd_desired is None else np.asarray(qd_desired, float)
        qdd_desired = np.zeros(n) if qdd_desired is None else np.asarray(qdd_desired, float)

        e = q_desired - self.robot.arm_q
        ed = qd_desired - self.robot.arm_qd

        qdd_cmd = qdd_desired + self.K_d * ed + self.K_p * e

        # M(q) q̈_cmd — the acceleration command priced in newton-metres at this
        # pose. Dropping M leaves the command *as if* every joint had unit
        # inertia, which is the break-it mode: the gains stop meaning anything
        # consistent, and the error grows wherever the real inertia is larger.
        M = self.inertia = self.arm_inertia()
        tau_inertial = M @ qdd_cmd if self.use_inertia else qdd_cmd.copy()

        gravity = self.gravity_torque()
        coriolis = self.bias_torque() - gravity

        tau_coriolis = coriolis if self.use_coriolis else np.zeros(n)
        tau_gravity = gravity if self.use_gravity else np.zeros(n)

        tau_demanded = tau_inertial + tau_coriolis + tau_gravity
        tau = np.clip(tau_demanded, self.torque_limits[:, 0], self.torque_limits[:, 1])
        self.saturated = tau_demanded != tau

        self.error, self.error_rate, self.qdd_cmd = e, ed, qdd_cmd
        self.tau_inertial = tau_inertial
        self.tau_coriolis = tau_coriolis
        self.tau_gravity = tau_gravity

        return tau, self._draw(q_desired)

    # --- the model, asked rather than written ------------------------------

    def arm_inertia(self) -> np.ndarray:
        """M(q) for the arm joints, (7, 7), in kg·m².

        `data.qM` is MuJoCo's sparse factorised form; `mj_fullM` expands it.
        The arm block is then sliced out by DOF address, which drops the rows
        and columns belonging to the fingers — they are commanded in metres by a
        position actuator, not accelerated by this law, and their inertial
        coupling into the arm is four orders of magnitude down.
        """
        mujoco.mj_fullM(self.robot.model, self._M_full, self.robot.data.qM)
        adr = self.robot.arm_dof_adr
        return self._M_full[np.ix_(adr, adr)]

    def bias_torque(self) -> np.ndarray:
        """h(q, q̇) = C(q, q̇) q̇ + g(q) on the arm joints, in N·m."""
        return self.robot.data.qfrc_bias[self.robot.arm_dof_adr].copy()

    def gravity_torque(self) -> np.ndarray:
        """g(q) alone, in N·m, by evaluating the bias at zero velocity.

        Deliberately a copy of what `gravity_comp.py` does rather than a call
        into it: the two rungs are meant to be readable one at a time, and this
        is six lines.
        """
        model, scratch = self.robot.model, self._scratch
        scratch.qpos[:] = self.robot.data.qpos
        scratch.qvel[:] = 0.0  # <- the line that separates g(q) from C(q, q̇) q̇
        mujoco.mj_kinematics(model, scratch)
        mujoco.mj_comPos(model, scratch)
        mujoco.mj_rne(model, scratch, 0, self._scratch_bias)
        return self._scratch_bias[self.robot.arm_dof_adr].copy()

    # --- drawing ----------------------------------------------------------

    def _draw(self, q_desired: np.ndarray) -> list:
        """The three pieces of the torque, and the pose they are chasing.

        One arrow per joint per piece, along that joint's own rotation axis, at
        the same scale as every other rung:

            violet          M(q) q̈_cmd, what it costs to accelerate this arm
                            from this pose. Watch it change size as the arm
                            extends without the command changing at all.
            cyan            C(q, q̇) q̇, the velocity-squared term. Invisible at
                            rest, and the whole lesson of the fast demo.
            wide orange     g(q), the same arrow rung 0 drew.

        and the pale skeleton is the pose the reference asks for.
        """
        primitives = [
            GhostPath(points=self._desired_skeleton(q_desired), width=0.012,
                      rgba=_GHOST_RGBA)
        ]

        anchors = self.robot.arm_joint_anchors
        axes = self.robot.arm_joint_axes

        for anchor, axis, inertial, coriolis, gravity in zip(
            anchors, axes, self.tau_inertial, self.tau_coriolis, self.tau_gravity
        ):
            for value, width, rgba in (
                (gravity, 0.022, _GRAVITY_RGBA),
                (coriolis, 0.014, _CORIOLIS_RGBA),
                (inertial, 0.008, _INERTIAL_RGBA),
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
        """Joint anchors of the desired pose, base to wrist, via forward
        kinematics on the scratch state."""
        scratch = self._scratch
        scratch.qpos[:] = self.robot.data.qpos
        scratch.qpos[self.robot.arm_qpos_adr] = q_desired
        mujoco.mj_kinematics(self.robot.model, scratch)
        return scratch.xanchor[self.robot.arm_joint_ids].copy()
