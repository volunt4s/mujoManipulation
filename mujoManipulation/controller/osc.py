"""Operational-space control — the rung that stops talking about joints.

Every law so far has been given seven numbers and asked to make seven numbers
match. But nobody wants a joint angle. What they want is the gripper *there*,
and the joints are whatever they have to be — which for a seven-jointed arm
reaching a six-dimensional pose is not even a unique answer.

So write the task where the task lives. With x the end-effector pose and
J(q) = ∂x/∂q the Jacobian that maps joint rates to end-effector rates:

    ẋ = J(q) q̇

Differentiate once and substitute the manipulator equation, and the arm's
apparent inertia *at the gripper* falls out:

    Λ(q) = (J M(q)⁻¹ Jᵀ)⁻¹

Λ is the mass the gripper has when you push it — six by six, pose-dependent,
and not diagonal: pushing the tip sideways makes it move forwards too, because
the arm behind it is not symmetric. Ask for a task-space acceleration, price it
through Λ, and send the wrench back down the Jacobian:

    a_cmd = ẍ_d + K_d ė + K_p e            e = x_d − x
    F     = Λ(q) a_cmd                     the wrench the task wants
    τ     = Jᵀ F  +  h(q, q̇)               the joint torques that deliver it

That is the whole law, and it is the first one on the ladder whose gains are in
metres and radians rather than joint angles: K_p is now the stiffness of the
gripper, in newtons per metre, and it means the same thing in every pose because
Λ absorbed the arm.

The null space, which is where the spare joint goes
    Jᵀ is seven by six. A seven-jointed arm holding a six-dimensional pose has
    one degree of freedom left over, and the law above says nothing about it —
    the elbow can swing through a whole arc without the gripper moving at all.
    Left alone it drifts, driven by whatever Coriolis and modelling error happen
    to push it, until it hits a joint limit.

    So the leftover is given a job, projected so that it cannot disturb the task:

        J̄ = M⁻¹ Jᵀ Λ                  the dynamically consistent inverse
        N = I − Jᵀ J̄ᵀ                 projects onto what the task does not use
        τ_0 = K_p,n (q_posture − q) − K_d,n q̇
        τ  +=  N τ_0

    `N` is built from M⁻¹ rather than from a plain pseudo-inverse on purpose: the
    plain one leaves a residual that accelerates the gripper, and the whole point
    is a posture that costs the task nothing. Switch the term off in the demo and
    the elbow wanders while the tip stays where it was put — which is the clearest
    picture of redundancy there is.

Where it goes wrong, and why the ellipsoid is drawn
    Λ is an inverse, and `J M⁻¹ Jᵀ` loses rank exactly when the arm runs out of
    ways to move the tip in some direction — a singularity. The wrench needed to
    accelerate along that direction goes to infinity, and the honest fix is to
    stop asking: the inverse is damped, which trades a little tracking for not
    launching the arm. The manipulability ellipsoid √det(J Jᵀ) drawn at the
    gripper is that margin made visible. It is a ball where the arm is dextrous
    and a flat disc when a direction is about to be lost, and the disc is normal
    to the direction you are about to lose.

The task arrives here as a joint pose, because that is what the rest of the
ladder is given and the comparison is only worth anything if everybody answers
the same question. Its end-effector frame — forward kinematics of it — is the
pose this law actually servos. Once a goal is specified in task space directly,
the joint-space rungs are the ones that need inverse kinematics to compete.

What is taken from MuJoCo and what is written here
    `mj_jacSite` and `mj_fullM` are the plant, like every other rung's model
    calls. Λ, the projector, the orientation error's logarithmic map and the
    damping are the algorithm, and they are written out.
"""

import mujoco
import numpy as np

from ..viz.primitives import Arrow, Ellipsoid

# One metre of arrow is this many newton-metres of joint torque. Shared with
# every other rung so the arrows stay comparable.
NM_PER_METRE = 40.0

# The task wrench is in newtons, not newton-metres, so it needs its own scale.
# Tied to the torque one by a constant so a single control moves both.
N_PER_NM = 2.0

# Semi-axis of the manipulability ellipsoid, in metres per (m/s per rad/s). The
# ellipsoid is a shape, not a measurement, and this only decides how big.
ELLIPSOID_SCALE = 0.11

# Λ is an inverse and the arm will be asked to work near a singularity. This is
# the λ in (J M⁻¹ Jᵀ + λI)⁻¹, in the units of inverse mass.
DAMPING = 0.02

# Public because the web layer sends these to the browser: a term's colour in
# the equation panel has to be the same colour as its arrow, and this is the
# only place that decides it.
WRENCH_RGBA = (0.95, 0.45, 0.75, 0.95)     # Jᵀ F, the task wrench
NULLSPACE_RGBA = (0.55, 0.85, 0.35, 0.85)  # N τ₀, the posture nobody sees
BIAS_RGBA = (0.95, 0.35, 0.15, 0.55)       # h(q, q̇), rung 0's colour again
ERROR_RGBA = (0.95, 0.85, 0.3, 0.95)       # x_d − x, the gap being closed
ELLIPSOID_RGBA = (0.3, 0.55, 0.95, 0.28)   # √det(J Jᵀ), the margin


def orientation_error(mat: np.ndarray, mat_d: np.ndarray) -> np.ndarray:
    """The rotation taking `mat` to `mat_d`, as a rotation vector.

    The logarithmic map of SO(3), written out because "the orientation error" is
    one of the things this rung exists to make concrete. R_err = R_d Rᵀ is the
    rotation still to be done; its axis and angle are read off the matrix:

        cos θ = (tr R_err − 1) / 2
        axis·sin θ = ½ [R₂₁−R₁₂, R₀₂−R₂₀, R₁₀−R₀₁]

    and the error is axis · θ, which is a vector in rad that a torque can be
    proportional to. The small-angle branch exists because axis·sinθ / sinθ is
    0/0 at θ = 0, and θ = 0 is where a working controller spends its time.
    """
    r_err = mat_d @ mat.T
    skew = np.array([
        r_err[2, 1] - r_err[1, 2],
        r_err[0, 2] - r_err[2, 0],
        r_err[1, 0] - r_err[0, 1],
    ])
    cos_theta = np.clip((np.trace(r_err) - 1.0) / 2.0, -1.0, 1.0)
    sin_theta = np.linalg.norm(skew) / 2.0

    if sin_theta < 1e-7:
        # Either no rotation at all, or exactly half a turn. Half a turn has no
        # unique axis sign anyway, and half a turn of error means the controller
        # has already lost.
        return 0.5 * skew if cos_theta > 0 else np.zeros(3)

    theta = np.arctan2(sin_theta, cos_theta)
    return skew / (2.0 * sin_theta) * theta


class OperationalSpaceControl:
    """OSC on the 7 arm joints, servoing the end-effector site.

    `K_p` / `K_d` are task-space gains: six numbers, or one scalar for all six,
    in N/m and N·s/m for the translational rows and their rotational equivalents
    for the last three. They mean the same thing at every pose, which is what Λ
    buys. `K_p_null` / `K_d_null` are joint-space, because the posture task is.
    """

    def __init__(
        self,
        robot,
        K_p=600.0,
        K_d=50.0,
        K_p_null=12.0,
        K_d_null=7.0,
        use_lambda: bool = True,
        use_nullspace: bool = True,
        use_bias: bool = True,
        damping: float = DAMPING,
        nm_per_metre: float = NM_PER_METRE,
    ):
        self.robot = robot
        n = robot.n_arm

        self.K_p = self._gains(K_p, 6, "K_p")
        self.K_d = self._gains(K_d, 6, "K_d")
        self.K_p_null = self._gains(K_p_null, n, "K_p_null")
        self.K_d_null = self._gains(K_d_null, n, "K_d_null")

        self.use_lambda = use_lambda
        self.use_nullspace = use_nullspace
        self.use_bias = use_bias
        self.damping = float(damping)
        self.nm_per_metre = nm_per_metre

        self.torque_limits = np.asarray(robot.arm_torque_limits, dtype=float)

        # Workspace. `_scratch` is for forward kinematics of the goal: the task
        # arrives as a joint pose and the frame it describes has to be asked for.
        self._M_full = np.zeros((robot.model.nv, robot.model.nv))
        self._jacp = np.zeros((3, robot.model.nv))
        self._jacr = np.zeros((3, robot.model.nv))
        self._scratch = mujoco.MjData(robot.model)
        self._scratch_jacp = np.zeros((3, robot.model.nv))
        self._scratch_jacr = np.zeros((3, robot.model.nv))

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
        """Clear the recorded quantities. There is no controller state."""
        n = self.robot.n_arm
        self.error = np.zeros(6)
        self.error_rate = np.zeros(6)
        self.wrench = np.zeros(6)
        self.tau_task = np.zeros(n)
        self.tau_null = np.zeros(n)
        self.tau_bias = np.zeros(n)
        self.target_pos = np.zeros(3)
        self.target_mat = np.eye(3)
        self.saturated = np.zeros(n, dtype=bool)
        self.manipulability = 0.0

    # --- the law ----------------------------------------------------------

    def update(self, q_desired, qd_desired=None, qdd_desired=None):
        """Returns (torque, draw primitives). The torque is 7 arm channels.

        The task is the end-effector frame of `q_desired`, and `qd_desired` —
        when given — becomes the task velocity through the Jacobian at the goal.
        `qdd_desired` is accepted and ignored: turning it into a task
        acceleration needs the J̇ q̇ term, which this rung does not carry.
        """
        n = self.robot.n_arm
        q_desired = np.asarray(q_desired, dtype=float)

        pos_d, mat_d, jac_d = self._goal_frame(q_desired)
        self.target_pos, self.target_mat = pos_d, mat_d

        J = self.jacobian()
        M = self.arm_inertia()
        M_inv = np.linalg.inv(M)

        # e, in metres and radians. The orientation half is the log map of the
        # rotation still to be done; nothing else here would tell you that a
        # "rotation error" is a vector at all.
        e = np.concatenate([
            pos_d - self.robot.ee_pos,
            orientation_error(self.robot.ee_mat, mat_d),
        ])
        xd_d = jac_d @ np.asarray(qd_desired, dtype=float) if qd_desired is not None \
            else np.zeros(6)
        ed = xd_d - J @ self.robot.arm_qd

        a_cmd = self.K_p * e + self.K_d * ed

        # Λ = (J M⁻¹ Jᵀ)⁻¹, damped. Dropping it is the break-it mode: the
        # command then goes down the Jacobian unpriced, which is
        # Jacobian-transpose control -- it still converges, and its stiffness
        # becomes a different number in every pose.
        lambda_inv = J @ M_inv @ J.T
        if self.use_lambda:
            Lambda = np.linalg.inv(lambda_inv + self.damping * np.eye(6))
            F = Lambda @ a_cmd
        else:
            Lambda = None
            F = a_cmd

        tau_task = J.T @ F
        tau_bias = self.bias_torque() if self.use_bias else np.zeros(n)
        tau_null = (
            self._nullspace(J, M_inv, Lambda, lambda_inv, q_desired)
            if self.use_nullspace else np.zeros(n)
        )

        tau_demanded = tau_task + tau_bias + tau_null
        tau = np.clip(tau_demanded, self.torque_limits[:, 0], self.torque_limits[:, 1])
        self.saturated = tau_demanded != tau

        self.error, self.error_rate, self.wrench = e, ed, F
        self.tau_task, self.tau_bias, self.tau_null = tau_task, tau_bias, tau_null
        self.manipulability = float(np.sqrt(max(np.linalg.det(J[:3] @ J[:3].T), 0.0)))

        return tau, self._draw(J)

    def _nullspace(self, J, M_inv, Lambda, lambda_inv, q_posture):
        """N τ₀ — the posture task, projected out of the end effector's way.

            J̄ = M⁻¹ Jᵀ Λ          N = I − Jᵀ J̄ᵀ

        With Λ switched off there is no dynamically consistent inverse to build
        the projector from, so the damped one stands in; the posture still works
        and is simply no longer free of charge.
        """
        if Lambda is None:
            Lambda = np.linalg.inv(lambda_inv + self.damping * np.eye(6))
        J_bar = M_inv @ J.T @ Lambda
        N = np.eye(self.robot.n_arm) - J.T @ J_bar.T
        tau_0 = (self.K_p_null * (q_posture - self.robot.arm_q)
                 - self.K_d_null * self.robot.arm_qd)
        return N @ tau_0

    # --- the model, asked rather than written ------------------------------

    def jacobian(self) -> np.ndarray:
        """J(q) at the end-effector site, (6, 7): three rows of ∂x/∂q and three
        of ∂ω/∂q, sliced to the arm's degrees of freedom."""
        mujoco.mj_jacSite(self.robot.model, self.robot.data,
                          self._jacp, self._jacr, self.robot.ee_site_id)
        adr = self.robot.arm_dof_adr
        return np.vstack([self._jacp[:, adr], self._jacr[:, adr]])

    def arm_inertia(self) -> np.ndarray:
        """M(q) for the arm joints, (7, 7)."""
        mujoco.mj_fullM(self.robot.model, self._M_full, self.robot.data.qM)
        adr = self.robot.arm_dof_adr
        return self._M_full[np.ix_(adr, adr)]

    def bias_torque(self) -> np.ndarray:
        """h(q, q̇) = C(q, q̇) q̇ + g(q) on the arm joints."""
        return self.robot.data.qfrc_bias[self.robot.arm_dof_adr].copy()

    def _goal_frame(self, q_desired):
        """Where the goal pose puts the end effector, and the Jacobian there.

        Forward kinematics on a throwaway MjData. The Jacobian at the *goal* is
        what turns a desired joint velocity into a desired task velocity, which
        is the only feedforward this rung has.
        """
        model, scratch = self.robot.model, self._scratch
        scratch.qpos[:] = self.robot.data.qpos
        scratch.qpos[self.robot.arm_qpos_adr] = q_desired
        scratch.qvel[:] = 0.0
        mujoco.mj_kinematics(model, scratch)
        mujoco.mj_comPos(model, scratch)
        mujoco.mj_jacSite(model, scratch, self._scratch_jacp, self._scratch_jacr,
                          self.robot.ee_site_id)
        adr = self.robot.arm_dof_adr
        site = self.robot.ee_site_id
        return (
            scratch.site_xpos[site].copy(),
            scratch.site_xmat[site].reshape(3, 3).copy(),
            np.vstack([self._scratch_jacp[:, adr], self._scratch_jacr[:, adr]]),
        )

    # --- drawing ----------------------------------------------------------

    def _draw(self, J) -> list:
        """What the law is made of, where it acts.

            yellow          x_d − x, from the gripper to where it was asked to
                            be. The one arrow that is a length rather than a
                            torque, and the one that should shrink to nothing.
                            The target *frame* is the caller's to draw.
            pink            Jᵀ F at each joint — the task wrench, priced by Λ and
                            pushed back down the arm.
            green           N τ₀ at each joint — the posture term. It should do
                            nothing to the tip, which is exactly what makes it
                            hard to see any other way.
            wide orange     h(q, q̇), the same arrow rung 0 drew.
            blue ellipsoid  √det(J Jᵀ) at the gripper: a ball where the arm is
                            dextrous, a disc where it is about to lose a
                            direction.
        """
        # The target frame itself is not drawn here. Where the task is, is a
        # fact about the task rather than a term of the law, so it belongs to
        # whoever is driving the loop -- which draws it for every rung alike and
        # lets it be switched off. `target_pos` / `target_mat` are published for
        # exactly that caller.
        primitives = [
            Arrow(origin=self.robot.ee_pos, vec=self.error[:3], width=0.009,
                  rgba=ERROR_RGBA),
        ]

        force_scale = self.nm_per_metre * N_PER_NM
        primitives.append(
            Arrow(origin=self.robot.ee_pos, vec=self.wrench[:3] / force_scale,
                  width=0.013, rgba=WRENCH_RGBA)
        )
        primitives.append(self._ellipsoid(J))

        anchors = self.robot.arm_joint_anchors
        axes = self.robot.arm_joint_axes
        for anchor, axis, bias, null, task in zip(
            anchors, axes, self.tau_bias, self.tau_null, self.tau_task
        ):
            for value, width, rgba in (
                (bias, 0.022, BIAS_RGBA),
                (null, 0.014, NULLSPACE_RGBA),
                (task, 0.008, WRENCH_RGBA),
            ):
                primitives.append(
                    Arrow(origin=anchor, vec=axis * (value / self.nm_per_metre),
                          width=width, rgba=rgba)
                )
        return primitives

    def _ellipsoid(self, J) -> Ellipsoid:
        """The translational manipulability ellipsoid at the gripper.

        `J_pos J_posᵀ` is symmetric positive semi-definite; its eigenvectors are
        the directions the tip can be moved and the square roots of its
        eigenvalues are how easily. Near a singularity one of them goes to zero
        and the ball becomes a disc.
        """
        values, vectors = np.linalg.eigh(J[:3] @ J[:3].T)
        radii = np.sqrt(np.clip(values, 0.0, None)) * ELLIPSOID_SCALE
        return Ellipsoid(center=self.robot.ee_pos, mat=vectors,
                         radii=np.maximum(radii, 1e-4), rgba=ELLIPSOID_RGBA)
