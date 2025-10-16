# Modular Robot Simulation
Modular robot simulation with MuJoCo

## Quick Start
Currently without ROS2 connection, just mujoco. Install mujoco via ```pip install mujoco``` (https://github.com/google-deepmind/mujoco)

Run
```
git clone git@github.com:twoo-kim/modular.git
cd src/controller/scripts
python3 test.py
```

## Explanation
1. MuJoCo xml composition
    Include environmental assets such as floor and sky in the ```environment.xml```
    Mesh assets and material features in ```asset.xml```
    Modular robot in ```modular.xml```
    - Mainbody-Cap-ServoMotor, motor_paddle-flA,B are rigidly attached
    - Two joints, motor and free-paddle joint
    - Only actuator apply force to the motor joint
    Multiple robots are declared and combined into a colony in ```test.xml```

2. Python
    Currently applying the same input force to each modular robots
    Going to implement simulator code that receives multiple control inputs from each modular robot (```simulator.py``` and ```agent.py```)

3. Fluid forces
   https://mujoco.readthedocs.io/en/stable/computation/fluid.html
   Mujoco just deals with fluid forces

## Future works
1. Check the relation of angles of each joint with the real experiment results
2. Check if the wrench is correct
