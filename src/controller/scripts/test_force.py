#!/usr/bin/env python3
import mujoco
import mujoco.viewer
import math
import time

import numpy as np
import matplotlib.pyplot as plt

# Open model
model = mujoco.MjModel.from_xml_path("/home/twkim/ros2_ws/src/modular/models/xml/test_force.xml")
data = mujoco.MjData(model)
i = 0

# Target ids
sensor_ids = []

# Read sensor data
for j in range(model.nsensor):
    name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_SENSOR, j)
    if 'force' in name:
        sensor_ids.append(j)

# Buffer
force = {}
t = []

step = 0.001


###############################  Run simulation  ###############################
with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        # Control; currently simple oscillation
        data.ctrl[:] = math.pi/4*math.sin(i/100*math.pi)
        i += 1
        
        # Forward step
        mujoco.mj_step(model, data)
        
        # Get force
        t.append(step*i)
        sensor_vals = data.sensordata
        for id in sensor_ids:
            f = sensor_vals[3*id:3*id+3]
            if id not in force:
                force[id] = {'x':[f[0]], 'y':[f[1]], 'z':[f[2]]}
            else:
                for n, elem in enumerate(force[id].values()):
                    elem.append(f[n])
        
        # Rendering
        viewer.sync()
        time.sleep(step)

        if i > 500:
            # Plot force - time
            body_id = sensor_ids[0]
            print(mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_SENSOR, body_id))
            fx = force[body_id]['x']
            fy = force[body_id]['y']
            fz = force[body_id]['z']
            plt.figure()
            plt.plot(t, fx, label="sensor x force")
            plt.plot(t, fy, label="sensor y force")
            plt.plot(t, fz, label="sensor z force")
            plt.xlabel("Time [s]")
            plt.ylabel("z-Force [N]")
            plt.title("Force vs Time")
            plt.legend()
            plt.grid(True)
            plt.show()
            
            # Impulse
            impulse = np.array([0.0, 0.0, 0.0])
            for idx in range(len(fx)):
                impulse += step*np.array([fx[idx], fy[idx], fz[idx]])
            print(f"Impulse (3D, N*t): {impulse}")

            break

