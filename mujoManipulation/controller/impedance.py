"""Impedance control — the rung that asks what happens when you push back.

Every law so far has had the same objective: make the error small. Each one got
better at it, and the last one got very good indeed. That objective has a cost
nobody has mentioned yet, which is that an arm which refuses to be wrong is an
arm which refuses to yield. Put a hand in its way and it will win.

So change the question. Not "how accurately does it track" but "how should it
*behave* when the world disagrees with it":

    e  =  x_d − x
    τ  =  Jᵀ [ K e + D ė ]  +  h(q, q̇)  +  N τ₀

which is a spring of stiffness K and a damper D, strung between the gripper and
where the gripper was asked to be, and nothing else. The arm becomes a physical
object with a compliance you chose. Push on it with F and it moves F/K — not
approximately, exactly, which the demo measures: 20 N against 150 N/m deflects
the tip 135 mm, against 600 N/m 32.7 mm, against 2400 N/m 8.3 mm. Hooke's law,
rendered by a torque controller.

What this is not, and the one line that says so
    `Λ(q)` is absent, on purpose. Operational-space control divides it out so
    that the gripper accelerates as commanded regardless of pose; here it is left
    in, so the arm keeps its own apparent mass and the only things specified are
    the stiffness and the damping. That is the whole difference between the two
    rungs, and it is one term.

    Shaping the inertia too — making the gripper *feel* like a mass you chose,
    not just a spring you chose — needs to know the external force, and knowing
    it needs a sensor. In simulation `xfrc_applied` is right there and it would
    be easy and dishonest to use; on hardware it is a force-torque sensor, a
    calibration and a filter. So the mass is the arm's, and this file says so
    rather than pretending.

The three terms, and what each is for
    K e         the spring. The stiffness is a specification, not a tuning: it
                is how hard the arm argues, in newtons per metre.
    D ė         the damper, and the one whose absence is loudest. A spring with
                no damper is a bell: push the arm, let go, and it rings for as
                long as the joint damping takes to give up.
    h(q, q̇)     gravity and Coriolis, as ever. Without it the spring hangs from
                a sagging anchor.
    N τ₀        the posture term, projected as in the previous rung, because
                seven joints holding a six-dimensional pose still have one
                joint spare.

Softness is the feature. A stiff arm is accurate and dangerous; a soft one is
safe and vague. The dial between them is K, it is in physical units, and this is
the rung where that trade is the subject rather than a side effect.
"""

import mujoco
import numpy as np

from ..viz.primitives import Arrow, GhostPath

# One metre of arrow is this many newton-metres. Shared with every other rung.
NM_PER_METRE = 40.0
N_PER_NM = 2.0

# The drawn spring: how many coils between the two ends, and how wide.
SPRING_COILS = 7
SPRING_RADIUS = 0.022
SPRING_POINTS = 64

DAMPING = 0.02

# Public because the web layer sends these to the browser: a term's colour in
# the equation panel has to be the same colour as its arrow, and this is the
# only place that decides it.
SPRING_RGBA = (0.35, 0.8, 0.95, 0.95)      # K e, drawn as an actual spring
DAMPER_RGBA = (0.95, 0.6, 0.15, 0.85)      # D ė
BIAS_RGBA = (0.95, 0.35, 0.15, 0.55)       # h(q, q̇), rung 0's colour again
NULLSPACE_RGBA = (0.55, 0.85, 0.35, 0.85)  # N τ₀


def spring_path(start, end, coils=SPRING_COILS, radius=SPRING_RADIUS,
                points=SPRING_POINTS) -> np.ndarray:
    """A helix from `start` to `end`, as a polyline.

    The README has promised since the beginning that impedance control would
    draw its virtual spring as an actual spring, and a helix of line segments is
    a `GhostPath` — so both renderers already know how to draw it and no new
    primitive is needed.

    The helix needs an axis perpendicular to the spring, and any choice is as
    good as another so long as it is not parallel to it; the world axis the
    spring is *least* aligned with is a stable one, where picking a fixed axis
    would collapse whenever the spring happened to point along it.
    """
    start = np.asarray(start, dtype=float)
    end = np.asarray(end, dtype=float)
    axis = end - start
    length = np.linalg.norm(axis)
    if length < 1e-6:
        return np.vstack([start, end])

    axis = axis / length
    reference = np.zeros(3)
    reference[np.argmin(np.abs(axis))] = 1.0
    u = np.cross(axis, reference)
    u /= np.linalg.norm(u)
    v = np.cross(axis, u)

    t = np.linspace(0.0, 1.0, points)
    # Taper the coils to nothing at both ends so the helix meets the two points
    # it is strung between instead of orbiting them.
    taper = np.sin(np.pi * t)
    angle = 2.0 * np.pi * coils * t
    offset = (radius * taper)[:, None] * (
        np.cos(angle)[:, None] * u + np.sin(angle)[:, None] * v
    )
    return start[None, :] + t[:, None] * (end - start)[None, :] + offset


class CartesianImpedance:
    """A spring and a damper at the end effector, and nothing else.

    `K_p` is a stiffness in N/m and `K_d` a damping in N·s/m — six numbers or one
    scalar. They are a specification of how the arm should behave, not gains to
    be tuned until the error goes away.
    """

    def __init__(
        self,
        robot,
        K_p=300.0,
        K_d=35.0,
        K_p_null=12.0,
        K_d_null=7.0,
        use_damping: bool = True,
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

        self.use_damping = use_damping
        self.use_nullspace = use_nullspace
        self.use_bias = use_bias
        self.damping = float(damping)
        self.nm_per_metre = nm_per_metre

        self.torque_limits = np.asarray(robot.arm_torque_limits, dtype=float)

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
        n = self.robot.n_arm
        self.error = np.zeros(6)
        self.error_rate = np.zeros(6)
        self.wrench_spring = np.zeros(6)
        self.wrench_damper = np.zeros(6)
        self.tau_spring = np.zeros(n)
        self.tau_damper = np.zeros(n)
        self.tau_bias = np.zeros(n)
        self.tau_null = np.zeros(n)
        self.target_pos = np.zeros(3)
        self.target_mat = np.eye(3)
        self.saturated = np.zeros(n, dtype=bool)

    # --- the law ----------------------------------------------------------

    def update(self, q_desired, qd_desired=None, qdd_desired=None):
        """Returns (torque, draw primitives). The torque is 7 arm channels.

            e = x_d − x
            τ = Jᵀ [K e + D ė] + h(q, q̇) + N τ₀

        The task is the end-effector frame of `q_desired`. `qdd_desired` is
        accepted and ignored: a spring has no use for a desired acceleration,
        which is the point of it.
        """
        from .osc import orientation_error      # the same log map, same reason

        n = self.robot.n_arm
        q_desired = np.asarray(q_desired, dtype=float)

        pos_d, mat_d, jac_d = self._goal_frame(q_desired)
        self.target_pos, self.target_mat = pos_d, mat_d

        J = self.jacobian()
        e = np.concatenate([
            pos_d - self.robot.ee_pos,
            orientation_error(self.robot.ee_mat, mat_d),
        ])
        xd_d = jac_d @ np.asarray(qd_desired, dtype=float) if qd_desired is not None \
            else np.zeros(6)
        ed = xd_d - J @ self.robot.arm_qd

        # The whole law. No Λ: the arm keeps its own apparent mass, and the
        # stiffness and damping are the only things specified.
        self.wrench_spring = self.K_p * e
        self.wrench_damper = self.K_d * ed if self.use_damping else np.zeros(6)

        tau_spring = J.T @ self.wrench_spring
        tau_damper = J.T @ self.wrench_damper
        tau_bias = self.bias_torque() if self.use_bias else np.zeros(n)
        tau_null = self._nullspace(J, q_desired) if self.use_nullspace else np.zeros(n)

        tau_demanded = tau_spring + tau_damper + tau_bias + tau_null
        tau = np.clip(tau_demanded, self.torque_limits[:, 0], self.torque_limits[:, 1])
        self.saturated = tau_demanded != tau

        self.error, self.error_rate = e, ed
        self.tau_spring, self.tau_damper = tau_spring, tau_damper
        self.tau_bias, self.tau_null = tau_bias, tau_null

        return tau, self._draw()

    def _nullspace(self, J, q_posture):
        """N τ₀, projected through the dynamically consistent inverse.

        Λ plays no part in the task here, but the projector still needs it: a
        posture that is not dynamically consistent pushes the gripper, and a
        spring that gets shoved by its own posture term is not a spring.
        """
        mujoco.mj_fullM(self.robot.model, self._M_full, self.robot.data.qM)
        adr = self.robot.arm_dof_adr
        M_inv = np.linalg.inv(self._M_full[np.ix_(adr, adr)])
        Lambda = np.linalg.inv(J @ M_inv @ J.T + self.damping * np.eye(6))
        N = np.eye(self.robot.n_arm) - J.T @ (M_inv @ J.T @ Lambda).T
        tau_0 = (self.K_p_null * (q_posture - self.robot.arm_q)
                 - self.K_d_null * self.robot.arm_qd)
        return N @ tau_0

    # --- the model, asked rather than written ------------------------------

    def jacobian(self) -> np.ndarray:
        mujoco.mj_jacSite(self.robot.model, self.robot.data,
                          self._jacp, self._jacr, self.robot.ee_site_id)
        adr = self.robot.arm_dof_adr
        return np.vstack([self._jacp[:, adr], self._jacr[:, adr]])

    def bias_torque(self) -> np.ndarray:
        return self.robot.data.qfrc_bias[self.robot.arm_dof_adr].copy()

    def _goal_frame(self, q_desired):
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

    def _draw(self) -> list:
        """The spring, as a spring.

            blue helix      K e, strung between the gripper and the pose it was
                            asked for. It stretches when the arm is pushed off
                            its target and coils back up as the arm returns,
                            which is the entire behaviour of this rung in one
                            object.
            orange          D ė at the gripper — only there while something is
                            moving, and the term whose absence makes the spring
                            a bell.
            wide orange     h(q, q̇) at each joint.
            green           N τ₀ at each joint.
        """
        force_scale = self.nm_per_metre * N_PER_NM
        primitives = [
            GhostPath(points=spring_path(self.robot.ee_pos, self.target_pos),
                      width=0.006, rgba=SPRING_RGBA),
            Arrow(origin=self.robot.ee_pos,
                  vec=self.wrench_damper[:3] / force_scale,
                  width=0.013, rgba=DAMPER_RGBA),
        ]

        anchors = self.robot.arm_joint_anchors
        axes = self.robot.arm_joint_axes
        for anchor, axis, bias, null, spring in zip(
            anchors, axes, self.tau_bias, self.tau_null, self.tau_spring
        ):
            for value, width, rgba in (
                (bias, 0.022, BIAS_RGBA),
                (null, 0.014, NULLSPACE_RGBA),
                (spring, 0.008, SPRING_RGBA),
            ):
                primitives.append(
                    Arrow(origin=anchor, vec=axis * (value / self.nm_per_metre),
                          width=width, rgba=rgba)
                )
        return primitives
