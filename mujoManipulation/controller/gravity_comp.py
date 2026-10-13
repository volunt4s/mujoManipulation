"""Gravity compensation — the first rung of the ladder.

The manipulator equation is

    M(q) q̈  +  C(q, q̇) q̇  +  g(q)  =  τ

Ask the arm to hold still. Then q̇ = 0 and q̈ = 0, two of the three terms on the
left vanish, and the whole thing collapses to

    τ = g(q)

which is this entire controller: ask the model what gravity is doing to each
joint, and send exactly that back. There is no gain to tune and no error to
measure — it is feedforward, and it is the reason every controller after this
one gets to pretend the arm is weightless.

Why not just use `data.qfrc_bias`?
    qfrc_bias is C(q, q̇) q̇ + g(q) — gravity *and* Coriolis, summed. Standing
    still the two are identical, so it would work here and hide the difference.
    Phase 1 has to switch the Coriolis term off on its own to show what it
    prevents, and it cannot do that if the two terms were never separated. So
    we evaluate the bias on a scratch MjData whose velocity is zero, which
    leaves g(q) by itself. One extra kinematics pass per step buys a term you
    can name.
"""

import mujoco
import numpy as np

from ..viz.primitives import Arrow

# Arrow length is torque / this, so one metre of arrow is this many newton-metres.
NM_PER_METRE = 40.0

_LOAD_RGBA = (0.95, 0.35, 0.15, 0.95)       # g(q): what gravity asks for
_RESIDUAL_RGBA = (0.95, 0.75, 0.15, 0.30)   # what it still gets, uncompensated


class GravityCompensation:
    def __init__(self, robot, enabled: bool = True, nm_per_metre: float = NM_PER_METRE):
        self.robot = robot
        self.enabled = enabled
        self.nm_per_metre = nm_per_metre

        # Scratch state used only to evaluate g(q). Never stepped, never
        # rendered — it exists so we can ask "what would the bias be if the arm
        # were standing still?" without disturbing the real sim.
        self._scratch = mujoco.MjData(robot.model)
        self._bias = np.zeros(robot.model.nv)

    def gravity_torque(self) -> np.ndarray:
        """g(q) on the arm joints, in N·m."""
        model = self.robot.model
        scratch = self._scratch

        scratch.qpos[:] = self.robot.data.qpos
        scratch.qvel[:] = 0.0  # <- the line that separates g(q) from C(q, q̇) q̇

        mujoco.mj_kinematics(model, scratch)
        mujoco.mj_comPos(model, scratch)
        mujoco.mj_rne(model, scratch, 0, self._bias)  # flg_acc = 0: no q̈ term

        return self._bias[self.robot.arm_dof_adr].copy()

    def update(self):
        """Returns (torque, draw primitives). The torque is 7 arm channels."""
        load = self.gravity_torque()
        tau = load.copy() if self.enabled else np.zeros_like(load)
        residual = load - tau
        return tau, self._draw(load, residual)

    def _draw(self, load: np.ndarray, residual: np.ndarray) -> list:
        """One arrow per joint along its own rotation axis, length proportional
        to torque and direction set by the sign — the right-hand rule, drawn.

        Two overlaid sets, and the contrast between them is the lesson:
          solid  g(q)      what gravity asks of this joint. A property of the
                           pose, so it does not change when you compensate.
          faint  residual  what gravity still gets. Compensation drives this to
                           zero; that is the arrow that disappears.
        """
        anchors = self.robot.arm_joint_anchors
        axes = self.robot.arm_joint_axes
        primitives = []

        for anchor, axis, load_i, residual_i in zip(anchors, axes, load, residual):
            primitives.append(
                Arrow(
                    origin=anchor,
                    vec=axis * (load_i / self.nm_per_metre),
                    width=0.008,
                    rgba=_LOAD_RGBA,
                )
            )
            primitives.append(
                Arrow(
                    origin=anchor,
                    vec=axis * (residual_i / self.nm_per_metre),
                    width=0.020,
                    rgba=_RESIDUAL_RGBA,
                )
            )
        return primitives
