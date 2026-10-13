"""MuJoCo model loading, introspection, stepping, and an optional viewer.

The sim loop lives in the caller, not in here. `MuJoCoEnv` owns the model, the
data, and (only if you ask for one) a passive viewer; it does not own the
control loop, a task, or a notion of episodes.

Headless is the default. Constructing an env must never open a window, because
everything above this file — tests, batch sweeps, the Phase 3 server — has to
be able to run without a display. Pass `render=True`, or call
`launch_viewer()` later, when you actually want to look at something.
"""

import os

import mujoco
import numpy as np


class MuJoCoEnv:
    def __init__(self, xml_path: str, render: bool = False, verbose: bool = False):
        self.xml_path = os.path.abspath(xml_path)
        self.verbose = verbose

        self.model = mujoco.MjModel.from_xml_path(self.xml_path)
        self.data = mujoco.MjData(self.model)

        self.viewer = None
        self.renderer = None

        self._introspect()
        if verbose:
            self.print_info()
        if render:
            self.launch_viewer()

    # --- model introspection ---------------------------------------------

    def _introspect(self) -> None:
        m = self.model

        self.n_dof = m.nv
        self.n_geom = m.ngeom
        self.n_body = m.nbody
        self.n_joint = m.njnt
        self.n_ctrl = m.nu

        self.geom_names = self._names(mujoco.mjtObj.mjOBJ_GEOM, m.ngeom)
        self.body_names = self._names(mujoco.mjtObj.mjOBJ_BODY, m.nbody)
        self.joint_names = self._names(mujoco.mjtObj.mjOBJ_JOINT, m.njnt)
        self.ctrl_names = self._names(mujoco.mjtObj.mjOBJ_ACTUATOR, m.nu)

        self.joint_types = m.jnt_type
        self.joint_ranges = m.jnt_range
        self.ctrl_ranges = m.actuator_ctrlrange

        # Joints grouped by kind. `rev_*` are the hinges you send torque to,
        # `pri_*` the sliders — on the Panda that is exactly arm vs. fingers.
        self._group_joints(mujoco.mjtJoint.mjJNT_HINGE, "rev", "revolute (hinge)")
        self._group_joints(mujoco.mjtJoint.mjJNT_SLIDE, "pri", "prismatic (slide)")

    def _names(self, obj_type, count) -> list:
        return [mujoco.mj_id2name(self.model, obj_type, i) for i in range(count)]

    def _group_joints(self, joint_type, prefix: str, description: str) -> None:
        idxs = np.where(self.joint_types == joint_type)[0].astype(np.int32)
        setattr(self, f"n_{prefix}_joint", len(idxs))
        setattr(self, f"{prefix}_joint_idxs", idxs)
        setattr(self, f"{prefix}_joint_names", [self.joint_names[i] for i in idxs])
        setattr(self, f"{prefix}_joint_mins", self.joint_ranges[idxs, 0])
        setattr(self, f"{prefix}_joint_maxs", self.joint_ranges[idxs, 1])
        setattr(
            self,
            f"{prefix}_joint_ranges",
            self.joint_ranges[idxs, 1] - self.joint_ranges[idxs, 0],
        )
        if self.verbose:
            print(f"Found {len(idxs)} {description} joint(s).")

    # --- stepping ---------------------------------------------------------

    @property
    def dt(self) -> float:
        return self.model.opt.timestep

    def step(self, n: int = 1) -> None:
        for _ in range(n):
            mujoco.mj_step(self.model, self.data)

    def forward(self) -> None:
        mujoco.mj_forward(self.model, self.data)

    def reset(self) -> None:
        mujoco.mj_resetData(self.model, self.data)
        self.forward()

    # --- viewer (optional) -------------------------------------------------

    def launch_viewer(self):
        """Open the passive viewer and wire the native renderer to it."""
        import mujoco.viewer

        from .viz.native import NativeRenderer

        self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
        self.renderer = NativeRenderer(self.viewer.user_scn)
        return self.viewer

    def is_running(self) -> bool:
        """True while the window is open. Always true when headless, so a
        headless caller must bound its own loop."""
        return True if self.viewer is None else self.viewer.is_running()

    def sync(self, primitives=()) -> None:
        """Push the frame, drawing `primitives` on top. A no-op when headless,
        so the same loop body works with and without a window."""
        if self.viewer is None:
            return
        if self.renderer is not None:
            self.renderer.draw(primitives)
        self.viewer.sync()

    def close(self) -> None:
        if self.viewer is not None:
            self.viewer.close()
            self.viewer = None
            self.renderer = None

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
        return False

    # --- reporting --------------------------------------------------------

    def print_info(self) -> None:
        line = "=" * 52
        print(f"\n{line}\n MODEL: {os.path.basename(self.xml_path)}\n{line}")
        print(f"  nq={self.model.nq}  nv={self.model.nv}  nu={self.model.nu}  "
              f"njnt={self.model.njnt}  timestep={self.dt}")
        print(f"\n  {'idx':>3}  {'joint':<22}{'actuator':<30}ctrlrange")
        for i in range(self.n_ctrl):
            joint_id = self.model.actuator_trnid[i, 0]
            joint = self.joint_names[joint_id] if joint_id >= 0 else "-"
            lo, hi = self.ctrl_ranges[i]
            print(f"  {i:>3}  {joint:<22}{self.ctrl_names[i]:<30}[{lo:g}, {hi:g}]")
        print(f"{line}\n")
