import mujoco
import mujoco.viewer
import numpy as np

from mujoManipulation.controller.PID import PIDController
from mujoManipulation.robot.Panda import FrankaPanda

panda_model = FrankaPanda(xml_path="/Users/hong/Desktop/github/mujoManipulation/mujoManipulation/assets/panda/franka_panda.xml")
m = panda_model.mj_model
d = panda_model.mj_data


controller = PIDController(K_p = 1000,
                           K_i = 0.1,
                           K_d = 50,
                           dt = 0.001,
                           tau = 0.002,
                           robot_model=panda_model)

desired_joint = panda_model.idle_pose

with mujoco.viewer.launch_passive(m, d) as viewer:
    while viewer.is_running():
        
        ut = controller.update(desired_joint, d.qpos)
        d.ctrl = ut
        mujoco.mj_step(m, d)
        viewer.sync()
