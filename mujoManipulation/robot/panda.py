"""Franka Emika Panda — a thin, named view over MjData.

The only thing this class really does is answer "which slot is that?". The
model hands you three flat 9-vectors (`qpos`, `qvel`, `ctrl`) whose meaning is
positional, and control code that indexes them with bare integers is unreadable
and silently wrong the day the XML changes.

The split that matters here is **arm vs. gripper**, because they are not the
same kind of channel:

    idx 0..6   panda_joint1..7        motor actuators, units are N·m
    idx 7..8   panda_finger_joint1,2  position actuators, units are metres

Sending a torque to a position actuator is meaningless, so nothing in this
class lets you address all nine at once. Indices are resolved by name at
construction, never hardcoded, which is what Phase 2 needs to swap in a UR5e
with nothing but a different name list.
"""

import mujoco
import numpy as np


class FrankaPanda:
    ARM_JOINTS = tuple(f"panda_joint{i}" for i in range(1, 8))
    GRIPPER_JOINTS = ("panda_finger_joint1", "panda_finger_joint2")

    # The frame the web viewer draws and the frame operational-space control
    # will servo. `grip_site` sits on `panda_eef`, between the fingers.
    EE_SITE = "grip_site"

    ARM_ACTUATORS = tuple(f"panda_torq_j{i}" for i in range(1, 8))
    GRIPPER_ACTUATORS = (
        "panda_gripper_finger_joint1",
        "panda_gripper_finger_joint2",
    )

    # Elbow-down pose the arm is asked to hold. Arm only — the gripper has its
    # own command because it is a different kind of channel.
    HOME = np.array(
        [0.0, -np.pi / 4, 0.0, -3 * np.pi / 4, 0.0, np.pi / 2, np.pi / 4]
    )
    GRIPPER_OPEN = 0.039

    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData):
        self.model = model
        self.data = data

        self.arm_joint_ids = self._ids(mujoco.mjtObj.mjOBJ_JOINT, self.ARM_JOINTS)
        self.gripper_joint_ids = self._ids(
            mujoco.mjtObj.mjOBJ_JOINT, self.GRIPPER_JOINTS
        )
        self.arm_act_ids = self._ids(
            mujoco.mjtObj.mjOBJ_ACTUATOR, self.ARM_ACTUATORS
        )
        self.gripper_act_ids = self._ids(
            mujoco.mjtObj.mjOBJ_ACTUATOR, self.GRIPPER_ACTUATORS
        )

        # Addresses into the flat state vectors. qpos and qvel are indexed
        # differently in general (a free joint is 7 wide in qpos, 6 in qvel),
        # so keep both even though every Panda joint happens to be 1-and-1.
        self.arm_qpos_adr = model.jnt_qposadr[self.arm_joint_ids]
        self.arm_dof_adr = model.jnt_dofadr[self.arm_joint_ids]
        self.gripper_qpos_adr = model.jnt_qposadr[self.gripper_joint_ids]

        self.ee_site_id = int(self._ids(mujoco.mjtObj.mjOBJ_SITE, (self.EE_SITE,))[0])

        self.n_arm = len(self.arm_joint_ids)
        self.arm_torque_limits = model.actuator_ctrlrange[self.arm_act_ids]

    def _ids(self, obj_type, names) -> np.ndarray:
        ids = [mujoco.mj_name2id(self.model, obj_type, n) for n in names]
        missing = [n for n, i in zip(names, ids) if i < 0]
        if missing:
            raise ValueError(
                f"model {self.model.names[:20]!r} is missing: {', '.join(missing)}"
            )
        return np.asarray(ids, dtype=np.int32)

    # --- arm state --------------------------------------------------------
    # Fancy indexing returns copies, not views into MjData. That is deliberate:
    # a controller holding a live alias to qpos is a bug waiting for the day
    # someone reorders the XML.

    @property
    def arm_q(self) -> np.ndarray:
        return self.data.qpos[self.arm_qpos_adr]

    @property
    def arm_qd(self) -> np.ndarray:
        return self.data.qvel[self.arm_dof_adr]

    @property
    def gripper_width(self) -> float:
        return float(self.data.qpos[self.gripper_qpos_adr[0]])

    # --- world-frame geometry, for drawing --------------------------------

    @property
    def ee_pos(self) -> np.ndarray:
        """(3,) world position of the end-effector site."""
        return self.data.site_xpos[self.ee_site_id].copy()

    @property
    def ee_mat(self) -> np.ndarray:
        """(3, 3) world orientation of the end-effector site, columns being its
        axes — the layout `viz.primitives.Frame` wants."""
        return self.data.site_xmat[self.ee_site_id].reshape(3, 3).copy()

    @property
    def arm_joint_anchors(self) -> np.ndarray:
        """(7, 3) world position of each arm joint's rotation axis."""
        return self.data.xanchor[self.arm_joint_ids]

    @property
    def arm_joint_axes(self) -> np.ndarray:
        """(7, 3) world direction of each arm joint's rotation axis."""
        return self.data.xaxis[self.arm_joint_ids]

    # --- commands ---------------------------------------------------------

    def set_arm_torque(self, tau: np.ndarray) -> None:
        tau = np.asarray(tau, dtype=float)
        if tau.shape != (self.n_arm,):
            raise ValueError(f"expected {self.n_arm} torques, got {tau.shape}")
        self.data.ctrl[self.arm_act_ids] = np.clip(
            tau, self.arm_torque_limits[:, 0], self.arm_torque_limits[:, 1]
        )

    def set_gripper(self, width: float) -> None:
        """Open to `width` metres. The two fingers mirror each other, which is
        why their ctrlranges have opposite signs."""
        self.data.ctrl[self.gripper_act_ids] = (width, -width)

    def go_home(self) -> None:
        """Teleport to the home pose. Sets state directly — this is for setup,
        not a controller."""
        self.data.qpos[self.arm_qpos_adr] = self.HOME
        self.data.qvel[self.arm_dof_adr] = 0.0
        self.data.qpos[self.gripper_qpos_adr] = (
            self.GRIPPER_OPEN,
            -self.GRIPPER_OPEN,
        )
        self.set_gripper(self.GRIPPER_OPEN)
        mujoco.mj_forward(self.model, self.data)
