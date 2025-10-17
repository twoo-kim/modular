#!/usr/bin/env python3
import mujoco
import mujoco.viewer
import math
import time
import os
import numpy as np
import matplotlib.pyplot as plt

# Open model
script_dir = os.path.dirname(os.path.abspath(__file__))
relative_path_to_model = os.path.join(script_dir, '../../../models/xml/test_force.xml')
model_path = os.path.abspath(relative_path_to_model)

model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)
i = 0

# Get initial configuration
key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "init_pose")
mujoco.mj_resetDataKeyframe(model, data, key_id)
mujoco.mj_forward(model, data)

# Target ids
robot_ids = []
motor_ids = []
paddle_ids = []
sensor_ids = []
for j in range(model.nbody):
    name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, j)
    if 'colony' or 'bot' in name:
        robot_ids.append(j)

for j in range(model.njnt):
    name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j)
    if 'motor' in name:
        motor_ids.append(j)
    elif 'paddle' in name:
        paddle_ids.append(j)

for j in range(model.nsensor):
    name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_SENSOR, j)
    if 'force' in name:
        sensor_ids.append(j)

# Buffer
ang_motor = {}
ang_paddle = {}
force = {}
orientation = {}
t = []
step = 0.001

def quaternion_to_matrix(q):
    w, x, y, z = q
    # normalize quaternion
    n = np.sqrt(x*x + y*y + z*z + w*w)
    w, x, y, z = w/n, x/n, y/n, z/n

    return np.array([
        [1 - 2*(y**2 + z**2),     2*(x*y - z*w),       2*(x*z + y*w)],
        [2*(x*y + z*w),           1 - 2*(x**2 + z**2), 2*(y*z - x*w)],
        [2*(x*z - y*w),           2*(y*z + x*w),       1 - 2*(x**2 + y**2)]
    ])

###############################  Run simulation  ###############################
with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        # Control; currently simple oscillation
        data.ctrl[:] = math.pi/4*math.sin(math.pi/2+i/100*math.pi)
        i += 1
        
        # Forward step
        mujoco.mj_step(model, data)
        
        # Get joint angles
        t.append(step*i)
        for id in motor_ids:
            qadr = model.jnt_qposadr[id]
            ang = data.qpos[qadr]
            if id not in ang_motor:
                ang_motor[id] = [ang]
            else:
                ang_motor[id].append(ang)
        
        for id in paddle_ids:
            qadr = model.jnt_qposadr[id]
            ang = data.qpos[qadr]
            if id not in ang_paddle:
                ang_paddle[id] = [ang]
            else:
                ang_paddle[id].append(ang)

        # Get orientation
        for idx, id in enumerate(robot_ids):
            orientation[idx] = quaternion_to_matrix(data.xquat[id])
        
        # Get force
        sensor_vals = np.array(data.sensordata)
        for idx, id in enumerate(sensor_ids):
            f = orientation[idx] @ sensor_vals[3*id:3*id+3]
            if id not in force:
                force[id] = {'x':[f[0]], 'y':[f[1]], 'z':[f[2]]}
            else:
                for n, elem in enumerate(force[id].values()):
                    elem.append(f[n])
        
        # Rendering
        viewer.sync()
        time.sleep(step)

        if i > 500:
            # Plot sensor angle - time
            a_motor = [180/math.pi*(i) for i in ang_motor[motor_ids[0]]]
            a_paddle = [180/math.pi*(i) for i in ang_paddle[paddle_ids[0]]]
            plt.figure()
            plt.plot(t, a_motor, label="motor_hinge angle")
            plt.plot(t, a_paddle, label="paddle_hinge angle")
            plt.xlabel("Time [s]")
            plt.ylabel("Angle [degree]")
            plt.title("Joint Angle vs Time")
            plt.legend()
            plt.grid(True)
            
            # Plot force - time
            body_id = sensor_ids[0]
            print(mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_SENSOR, body_id))
            fx = force[body_id]['x']
            fy = force[body_id]['y']
            fz = force[body_id]['z']
            plt.figure()
            plt.plot(t, fx, label="COM x force")
            plt.plot(t, fy, label="COM y force")
            plt.plot(t, fz, label="COM z force")
            plt.xlabel("Time [s]")
            plt.ylabel("z-Force [N]")
            plt.title("Force vs Time")
            plt.legend()
            plt.grid(True)
            plt.show()
            
            # Impulse
            impulse = np.array([0.0, 0.0, 0.0])
            v = np.array([0.0, 0.0, 0.0])
            x = np.array([0.0, 0.0, 0.5])
            mass = np.sum(model.body_mass)
            for idx in range(len(fx)):
                f_vec = np.array([fx[idx], fy[idx], fz[idx]])
                impulse += step*f_vec
                a = f_vec/mass
                v += a*step
                x += v*step + 0.5*a*step**2
            
            # Actual position
            base_id = model.body("colony").id
            position = np.array([data.xpos[base_id][0], data.xpos[base_id][1], data.xpos[base_id][2]])

            base_twist = data.cvel[base_id]
            vel = np.array([base_twist[3], base_twist[4], base_twist[5]])
        
            print(f"Mass(kg): {mass}")
            print(f"Impulse(N*t): {impulse}")
            print(f"Expected position(m): {x}")
            print(f"Actual position(m): {position}")
            print(f"Expected velocity(m/s): {impulse/mass}")
            print(f"Actual velocity(m/s): {vel}")
            break

