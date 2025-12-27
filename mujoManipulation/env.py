import os
import mujoco
import mujoco.viewer
import numpy as np

class MuJoCoEnv:
    def __init__(self, xml_path, verbose=False):
        self.xml_path = xml_path
        self._parse_xml()
        self._init_viewer()

        if verbose:
            self.print_info()

    def _parse_xml(self):
        """
        Parse MuJoCo xml to MuJoCo model info
        """
        print(f"Parsing XML from: {self.xml_path}")
        self.full_xml_path = os.path.abspath(os.path.join(os.getcwd(), self.xml_path))
        
        try:
            self.model = mujoco.MjModel.from_xml_path(self.full_xml_path)
            self.data = mujoco.MjData(self.model)
        except Exception as e:
            print(f"Error loading MuJoCo model: {e}")
            raise

        # Geometry, body
        self.n_dof = self.model.nv
        self.n_geom = self.model.ngeom
        self.geom_names = [
            mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, i)
            for i in range(self.n_geom)
        ]
        self.n_body = self.model.nbody
        self.body_names = [
            mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, i)
            for i in range(self.n_body)
        ]
        
        # Joint
        self.n_joint = self.model.njnt
        self.joint_types = self.model.jnt_type
        self.joint_ranges = self.model.jnt_range

        self.joint_names = [
            mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, i)
            for i in range(self.n_joint)
        ]
        self._parse_specific_joints(
            joint_type=mujoco.mjtJoint.mjJNT_HINGE,
            prefix="rev",
            description="Revolute (HINGE) Joint"
        )
        self._parse_specific_joints(
            joint_type=mujoco.mjtJoint.mjJNT_SLIDE,
            prefix="pri",
            description="Prismatic (SLIDE) Joint"
        )

        # Actuator
        self.n_ctrl = self.model.nu
        self.ctrl_names = [
            self.model.names[addr:].decode().split('\x00')[0]
            for addr in self.model.name_actuatoradr
        ]
        self.ctrl_joint_idxs = [
            self.model.actuator(name).trnid
            for name in self.ctrl_names
        ]
        self.ctrl_ranges = self.model.actuator_ctrlrange # Control range [min, max]
        
        print("MuJoCo model loaded!")

    def _parse_specific_joints(self, joint_type, prefix, description):
        idxs = np.where(self.joint_types == joint_type)[0].astype(np.int32)
        n_joint = len(idxs)

        setattr(self, f"n_{prefix}_joint", n_joint)
        setattr(self, f"{prefix}_joint_idxs", idxs)
        
        print(f"Found {n_joint} {description}s.")

        if n_joint > 0:
            names = [self.joint_names[i] for i in idxs]

            mins = self.joint_ranges[idxs, 0]
            maxs = self.joint_ranges[idxs, 1]
            ranges = maxs - mins

            setattr(self, f"{prefix}_joint_names", names)
            setattr(self, f"{prefix}_joint_mins", mins)
            setattr(self, f"{prefix}_joint_maxs", maxs)
            setattr(self, f"{prefix}_joint_ranges", ranges)

        else:
            setattr(self, f"{prefix}_joint_names", [])
            setattr(self, f"{prefix}_joint_mins", np.array([]))
            setattr(self, f"{prefix}_joint_maxs", np.array([]))
            setattr(self, f"{prefix}_joint_ranges", np.array([]))

    def _init_viewer(self):
        self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
    
    def step(self):
        mujoco.mj_step(self.model, self.data)

    def print_info(self):
        """
            Printout model information.
        """
        separator = "=" * 40
        print(f"\n{separator}")
        print("🤖 MODEL INFORMATION SUMMARY")
        print(f"{separator}")

        print("\n--- General Counts ---")
        print(f"Number of Geometries (n_geom): {self.n_geom}")
        print(f"Geometry Names (geom_names): {self.geom_names}")
        print(f"Number of Bodies (n_body): {self.n_body}")
        print(f"Body Names (body_names): {self.body_names}")

        print("\n--- Control Counts ---")
        print(f"Number of Controllers (n_ctrl): {self.n_ctrl}")
        print(f"Controller Names (ctrl_names): {self.ctrl_names}")
        print(f"Controller Joint Indices (ctrl_joint_idxs): {self.ctrl_joint_idxs}")
        print(f"Controller Ranges (ctrl_ranges):\n{self.ctrl_ranges}")
        
        print("\n--- Joint Information ---")
        print(f"Total Number of Joints (n_joint): {self.n_joint}")
        print(f"Joint Names (joint_names): {self.joint_names}")
        print(f"Joint Types (joint_types): {self.joint_types}")
        print(f"Joint Ranges (joint_ranges):\n{self.joint_ranges}")
        
        print("\n--- Revolute Joints ---")
        print(f"Number of Revolute Joints (n_rev_joint): {self.n_rev_joint}")
        if self.n_rev_joint > 0:
            print(f"Indices (rev_joint_idxs): {self.rev_joint_idxs}")
            print(f"Names (rev_joint_names): {self.rev_joint_names}")
            print(f"Mins (rev_joint_mins): {self.rev_joint_mins}")
            print(f"Maxs (rev_joint_maxs): {self.rev_joint_maxs}")
            print(f"Ranges (rev_joint_ranges): {self.rev_joint_ranges}")

        print("\n--- Prismatic Joints ---")
        print(f"Number of Prismatic Joints (n_pri_joint): {self.n_pri_joint}")
        if self.n_pri_joint > 0:
            print(f"Indices (pri_joint_idxs): {self.pri_joint_idxs}")
            print(f"Names (pri_joint_names): {self.pri_joint_names}")
            print(f"Mins (pri_joint_mins): {self.pri_joint_mins}")
            print(f"Maxs (pri_joint_maxs): {self.pri_joint_maxs}")
            print(f"Ranges (pri_joint_ranges): {self.pri_joint_ranges}")
            
        print(f"\n{separator}\n")