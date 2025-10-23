#!/usr/bin/env python3
import mujoco
import mujoco.viewer
import math
import time
import os
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Open model
<<<<<<< HEAD
script_dir = os.path.dirname(os.path.abspath(__file__))
relative_path_to_model = os.path.join(script_dir, '../../../models/xml/test_force.xml')
model_path = os.path.abspath(relative_path_to_model)

# Open model
model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)
i = 0
=======
model = mujoco.MjModel.from_xml_path("/home/twkim/ros2_ws/src/modular/models/xml/test_force.xml")
>>>>>>> cedb6d0 (simulation node with circular colony)

############################### Simulation Parameter ###############################
step = 0.001        # s (if change this value, change xml timestep also)
sample_time = 10    # s
t = [step*i for i in range(int(sample_time/step))]

####################################################################################
def run_episode(model, amplitude, frequency):
    data = mujoco.MjData(model)
    # Target ids
    sensor_ids = []

    # Read sensor data
    for j in range(model.nsensor):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_SENSOR, j)
        if 'force' in name:
            sensor_ids.append(j)

    # Episode parameters
    amp = amplitude
    freq = frequency
    omega = 2*math.pi*freq*step

    # Run simulation
    fz = []
    for i in range(int(sample_time/step)):
        data.ctrl[:] = amp*math.sin(omega*i)
        mujoco.mj_step(model, data)

        # Get force
        sensor_vals = data.sensordata
        for id in sensor_ids:
            fz.append(sensor_vals[3*id:3*id+3][2])
    
    impulse = step*sum(fz)
    average_fz = impulse/sample_time
    return average_fz

##### Run episodes ####
angles = np.array([5*i+20 for i in range(10)])
frequencies = np.array([0.25*i+0.75 for i in range(5)])
result = {}
Z = np.zeros((len(frequencies), len(angles)))
for i, freq in enumerate(frequencies):
    for j, ang in enumerate(angles):
        value = run_episode(model, ang*math.pi/180, freq)
        if freq not in result:
            result[freq] = [(ang, value)]
        else:
            result[freq].append((ang, value))
        Z[i, j] = value

X, Y = np.meshgrid(angles, frequencies)
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot_surface(X, Y, Z, linewidth=0, antialiased=True)

ax.set_xlabel('Angle (deg)')
ax.set_ylabel('Frequency (Hz)')
ax.set_zlabel('Result value')

# Hz, angle force relation
plt.figure()
for freq in result.keys():
    ang = [f[0] for f in result[freq]]
    frc = [f[1] for f in result[freq]]
    plt.plot(ang, frc, label=f"{freq}Hz")
plt.xlabel("Angle [degree]")
plt.ylabel("Average force [N]")
plt.title("Average force - Angle")
plt.legend()
plt.grid(True)
plt.show()

# # Buffer
# force = {}
# t = []

# step = 0.001

# ###############################  Run simulation  ###############################
# with mujoco.viewer.launch_passive(model, data) as viewer:
#     while viewer.is_running():
#         # Control; currently simple oscillation
#         data.ctrl[:] = amp*math.sin(omega*i)
#         i += 1
        
#         # Forward step
#         mujoco.mj_step(model, data)

#         # Get force
#         t.append(step*i)
#         sensor_vals = data.sensordata
#         for id in sensor_ids:
#             f = sensor_vals[3*id:3*id+3]
#             if id not in force:
#                 force[id] = {'x':[f[0]], 'y':[f[1]], 'z':[f[2]]}
#             else:
#                 for n, elem in enumerate(force[id].values()):
#                     elem.append(f[n])
        
#         # Rendering
#         viewer.sync()
#         time.sleep(step)

#         if i > 500:
#             # Plot force - time
#             body_id = sensor_ids[0]
#             print(mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_SENSOR, body_id))
#             fx = force[body_id]['x']
#             fy = force[body_id]['y']
#             fz = force[body_id]['z']
#             plt.figure()
#             plt.plot(t, fx, label="COM x force")
#             plt.plot(t, fy, label="COM y force")
#             plt.plot(t, fz, label="COM z force")
#             plt.xlabel("Time [s]")
#             plt.ylabel("z-Force [N]")
#             plt.title("Force vs Time")
#             plt.legend()
#             plt.grid(True)
#             plt.show()
            
#             # Impulse
#             impulse = np.array([0.0, 0.0, 0.0])
#             for idx in range(len(fx)):
#                 impulse += step*np.array([fx[idx], fy[idx], fz[idx]])
#             print(f"Impulse (3D, N*t): {impulse}")

#             break
