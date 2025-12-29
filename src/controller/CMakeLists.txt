cmake_minimum_required(VERSION 3.8)
project(modular)

# Required version
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE RelWithDebInfo) # Release, RelWithDebInfo, Debug

# Find ROS2 packages; REQUIRED for immediate error detection
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(modular_msgs REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(glfw3 REQUIRED)

set(Python3_EXECUTABLE /usr/bin/python3 CACHE FILEPATH "" FORCE)
find_package(Python3 COMPONENTS Interpreter Development NumPy REQUIRED)
# find_package(pinocchio REQUIRED)

# matplotlib-cpp
set(MATPLOTLIB_CPP_DIR ${CMAKE_SOURCE_DIR}/../external/matplotlib-cpp)

# Mujoco
set(MUJOCO_PATH "~/.mujoco/mujoco-3.3.7")
include_directories(include)
include_directories(${MUJOCO_PATH}/include/mujoco)
include_directories(${MUJOCO_PATH}/include)
link_directories(${MUJOCO_PATH}/lib)

# ------------------------------------------------------------------------------ #
# Library;  compile library into .a or .so 
add_library(modular_library
  src/controller/module_ctrl.cpp
  src/data/config.cpp
  src/simulation/mujoco_simulator.cpp
  src/simulation/viewer.cpp
)

add_library(dynamics_library
  src/data/config.cpp
  src/simulation/dynamics_simulator.cpp
  src/simulation/viewer.cpp
)

target_include_directories(modular_library PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_include_directories(dynamics_library PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${MATPLOTLIB_CPP_DIR}
)

target_link_libraries(modular_library
  Eigen3::Eigen
  yaml-cpp
  mujoco
  glfw
)

target_link_libraries(dynamics_library
  Eigen3::Eigen
  yaml-cpp
  mujoco
  glfw
  # pinocchio::pinocchio
)

ament_target_dependencies(modular_library
  rclcpp
  std_msgs
  geometry_msgs
  modular_msgs
)

ament_target_dependencies(dynamics_library
  std_msgs
  geometry_msgs
)

# ------------------------------------------------------------------------------ #
# Executable; compile into object file and tell linker to include library
add_executable(ctrl_node src/ctrl_node.cpp)
target_link_libraries(ctrl_node modular_library)
ament_target_dependencies(ctrl_node
  rclcpp
  std_msgs
  geometry_msgs
  modular_msgs
)

add_executable(sim_node src/sim_node.cpp)
target_link_libraries(sim_node modular_library)
ament_target_dependencies(sim_node
  rclcpp
  std_msgs
  geometry_msgs
  modular_msgs
)

add_executable(colony_sim src/dynamics_simulation.cpp)
target_include_directories(colony_sim PRIVATE
  ${MATPLOTLIB_CPP_DIR}
)
target_link_libraries(colony_sim PRIVATE
  dynamics_library
  Python3::Python
  Python3::NumPy
)

# ------------------------------------------------------------------------------ #
# Install; locate compiled files into the destination
install(TARGETS
  ctrl_node
  sim_node
  colony_sim
  modular_library
  dynamics_library
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

# Python simulator; PROGRAMS stands for executable scripts such as Python
install(PROGRAMS
  scripts/test_module.py
  scripts/test_force.py
  DESTINATION lib/${PROJECT_NAME}
)

# Launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Config and model files
install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config
)
install(DIRECTORY
  ../../models/xml/
  ../../models/urdf/
  DESTINATION share/${PROJECT_NAME}/models
)

# ------------------------------------------------------------------------------ #
# Export interfaces
ament_export_include_directories(include)
ament_export_libraries(modular_library dynamics_library)
ament_export_dependencies(rclcpp std_msgs geometry_msgs modular_msgs)

ament_package()