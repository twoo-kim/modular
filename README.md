# Modular Robot Simulation
Modular robot simulation with MuJoCo

## Quick Start
For Python, simply ```pip install mujoco```  
For the ROS2 integration simulations with C++ need to install mujoco binaries  
1. **Install MuJoCo**:  
  
    Download and install MuJoCo from https://github.com/google-deepmind/mujoco/releases

    I have installed the latest `mujoco-3.3.7`
  
    Next, we need to export environment variable for compile `vi ~/.bashrc`

    And add the following lines to the bash file
      ```
      export MUJOCO_PATH=~/.mujoco/mujoco-3.3.7
      export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/.mujoco/mujoco-3.3.7/bin
      export LD_LIBRARY_PATH=~/.mujoco/mujoco-3.3.7/lib:$LD_LIBRARY_PATH
      ```

2. **Install GLFW3**:
    
    MuJoCo utilize this for visualization. `sudo apt-get install libglfw3-dev`  

3. **Compile and Run**:  
    ```
    git clone git@github.com:twoo-kim/modular.git
    ```
    For python  
    ```
    cd src/controller/scripts
    python3 test.py
    ```
    For C++ node, go to the ```ros2_ws``` workspace and  
    ```
    colcon build --packages-up-to modular
    ros2 launch modular modular_launch.py
    ```
  The structure of this repository is as below  
  ```
  modular/
  ├── models/
  ├── modular_msgs/
  └── src/
      └── controller/
          ├── config/
          ├── include/
          ├── launch/
          ├── scripts/
          ├── src/
          ├── CMakeLists.txt
          └── package.xml
  ```

## Explanation
**1. MuJoCo xml**  
  ```environment.xml```: sky, floor, ...  
  ```asset.xml```: Mesh, material features  
  ```modular.xml```: Main module  
  ```test_force.xml```: Force sensor test used for ```test_force.py```  
  ```test_module.xml```: Module test used for ```test_module.py```
  ```models/xml/colony/*.xml```: xml files with circular colony made by simulator node  

**2. Python**  
  ```test_colony.py```: Colony test file; not modified  
  ```test_force.py```: Single paddle force test  
  ```test_module.py```: Simple multiple module test  

**3. C++ Simulation**  
  - **YAML**:
  The simulation node reads YAML file so that we can adjust parameters easily without frequent compiles.

  - **ROS msg**
    
    ```StateMsg.msg```: State message that informs values from mujoco to each modules. Just store the values as the same order of the index, first find the matching ```module_id``` from ```index``` and extract the data at the same location
    
    ```ControlMsg.msg```: Advertise its index, control input, internal phase  
  - **Headers**  
    ```module_ctrl.hpp```: Simple controller node that publishes control  
    ```data/config.hpp```: Configuration file for simulation  
    ```data/module.hpp```: Structure for storing module ids  
    ```simulation/viewer.hpp```: Viewer that visualize mujoco environment  
    ```simulation/simulator.hpp```: Simulator node that subscribes control and process steps  
    ```utils/utils.hpp```: Utility function used for type conversion  
    ```utils/xml_utils.hpp```: Utility function used for xml generation, e.g. circular colony  

  - **Launch**

    Generate N number of module controllers and a single simulator node

## Future works
1. Currently, I just simply modified ```test_colony.py``, so need to be modifed to make it more user-friendly
2. Modify the controller node to be synchronized  

