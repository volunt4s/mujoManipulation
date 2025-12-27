import mujoco
import numpy as np

class FrankaPanda:
    def __init__(self,
                 model: mujoco.MjModel,
                 data : mujoco.MjData):
        self.model = model
        self.data = data
        self.idle_joint = np.array([0, -0.785398163397, 0, -2.35619449019, 0, 1.57079632679, 0.785398163397, 0.039, -0.039])
        self.desired_joint = 0.0
        self.joint_value = self.data.qpos

    def control(self,
                torque):
        self.data.ctrl = torque