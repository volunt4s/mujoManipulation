import os
from mujoManipulation.env import MuJoCoEnv
from mujoManipulation.robot.panda import FrankaPanda
from mujoManipulation.controller.pid import PIDController

if __name__ == "__main__":
    xml_path_lst = ["mujoManipulation", "assets", "panda", "franka_panda.xml"]
    xml_path = os.path.abspath(os.path.join(*xml_path_lst))
    env = MuJoCoEnv(xml_path=xml_path,
                    verbose=False)
    panda = FrankaPanda(model=env.model,
                        data=env.data)
    controller = PIDController(K_p=1000,
                               K_i=10,
                               K_d=50,
                               dt=0.001,
                               tau=0.002,
                               robot=panda)
    
    while env.viewer.is_running():
        ut = controller.update(desired=panda.idle_joint,
                               curr=panda.joint_value)
        panda.control(ut)
        env.step()
        env.viewer.sync()
        