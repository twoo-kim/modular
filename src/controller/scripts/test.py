#!/usr/bin/env python3
import mujoco
import mujoco.viewer
import math
import time

import numpy as np

model = mujoco.MjModel.from_xml_path("/home/twkim/ros2_ws/src/modular/models/xml/test.xml")
data = mujoco.MjData(model)
i = 0

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        # Control; currently simple oscillation
        data.ctrl[:] = 0.5*math.sin(i/100*math.pi)
        i += 1
        
        # Forward step
        mujoco.mj_step(model, data)
        
        # Read sensor data
        # sensor = {}
        # sensor_vals = data.sensordata
        # for j in range(model.nsensor):
        #     name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_SENSOR, j)
        #     sensor[name] = sensor_vals[3*j:3*j+3]
        
        # for k in range(model.njnt):
        #     name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, k)
        #     print(name, k)
        
        # for name in sensor:
        #     if 'force' in name:
        #         print(f"{name}: {sensor[name]}")

        # Rendering
        viewer.sync()
        time.sleep(0.001)
