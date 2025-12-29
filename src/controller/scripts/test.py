import mujoco
import mujoco.viewer
import os

urdf_path = "/home/twkim/ros2_ws/src/modular/models/meshes/colony.urdf"

model = mujoco.MjModel.from_xml_path(urdf_path)
data = mujoco.MjData(model)
model.opt.gravity[:] = 0.0


with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        mujoco.mj_step(model, data)
        viewer.sync()