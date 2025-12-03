import mujoco
import numpy as np

class FrankaPanda:
    def __init__(self,
                 xml_path):
        self.xml_path = xml_path

        self.mj_model = mujoco.MjModel.from_xml_path(xml_path)
        self.mj_data = mujoco.MjData(self.mj_model)

        self.ctrl_range = self.mj_model.actuator_ctrlrange
        self.dof = self.mj_model.nv
        self.idle_pose = np.array([0, -0.785398163397, 0, -2.35619449019, 0, 1.57079632679, 0.785398163397, 0.039, -0.039])