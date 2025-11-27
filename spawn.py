import mujoco
import mujoco.viewer
import numpy as np

from mujoManipulation.controller.PID import PIDController

m = mujoco.MjModel.from_xml_path("/Users/hong/Desktop/github/mujoManipulation/mujoManipulation/assets/panda/franka_panda.xml")
d = mujoco.MjData(m)

controller = PIDController(K_p = 1000,
                           K_i = 0,
                           K_d = 50,
                           dt = 0.001)


desired_joint = np.array([0, -0.785398163397, 0, -2.35619449019, 0, 1.57079632679, 0.785398163397, 0, 0])
ut = controller.update(desired_joint, d.qpos)

print(ut)

with mujoco.viewer.launch_passive(m, d) as viewer:
    while viewer.is_running():
        
        ut = controller.update(desired_joint, d.qpos)
        print(ut)
        d.ctrl = ut
        mujoco.mj_step(m, d)
        viewer.sync()
        