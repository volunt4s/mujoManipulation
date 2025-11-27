import mujoco
import mujoco.viewer

m = mujoco.MjModel.from_xml_path("/Users/hong/Desktop/github/mujoManipulation/mujoManipulation/assets/panda/franka_panda.xml")
d = mujoco.MjData(m)

with mujoco.viewer.launch(m, d) as viewer:
    while viewer.is_running():
        viewer.sync()