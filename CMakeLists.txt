cmake_minimum_required(VERSION 3.8)
project(am_controller)

# Required version
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Not using pinocchio for python
set(BUILD_WITH_PYTHON_INTERFACE OFF)
set(PINOCCHIO_WITH_PYTHON_INTERFACE OFF)

# Compile with Release
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "RelWithDebInfo" "MinSizeRel")
endif()

# For debug
# if(NOT CMAKE_CONFIGURATION_TYPES)
#   if(NOT CMAKE_BUILD_TYPE)
#     set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
#     set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "RelWithDebInfo" "MinSizeRel")
#   endif()
# endif()

# Find ROS2 packages; REQUIRED for immediate error detection
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(am_msgs REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
find_package(yaml-cpp REQUIRED)

# ------------------------------------------------------------------------------ #
# Library;  compile library into .a or .so 
add_library(am_library
  src/controller/controller_base.cpp
  src/controller/am_mpc.cpp
  src/controller/am_geometric.cpp
  src/solver/DDP.cpp
  src/solver/FDDP.cpp
  src/solver/BoxQP.cpp
  src/solver/BoxDDP.cpp
  src/solver/BoxFDDP.cpp
  src/model/cost/activation.cpp
  src/model/cost/cost_model.cpp
  src/model/state/state.cpp
  src/model/state/ndob.cpp
  src/model/model/robot_model.cpp
  src/model/model/am_model.cpp
  src/model/model/integrated_model.cpp
  src/data/config.cpp
)

target_include_directories(am_library PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(am_library
  Eigen3::Eigen
  pinocchio::pinocchio
  yaml-cpp
)

ament_target_dependencies(am_library
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  am_msgs
)

# ------------------------------------------------------------------------------ #
# Executable; compile into object file and tell linker to include library
add_executable(mpc_node src/mpc_node.cpp)
target_link_libraries(mpc_node am_library)
ament_target_dependencies(mpc_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  am_msgs
)

add_executable(geo_node src/geo_node.cpp)
target_link_libraries(geo_node am_library)
ament_target_dependencies(geo_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  am_msgs
)

# ------------------------------------------------------------------------------ #
# Install; locate compiled files into the destination
install(TARGETS
  mpc_node
  geo_node
  am_library
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

# Python simulator; PROGRAMS stands for executable scripts such as Python
install(PROGRAMS
  scripts/simulator.py
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
  ../../models/urdf/
  ../../models/xml/
  DESTINATION share/${PROJECT_NAME}/models
)

# ------------------------------------------------------------------------------ #
# Export interfaces
ament_export_include_directories(include)
ament_export_libraries(am_library)
ament_export_dependencies(rclcpp std_msgs geometry_msgs am_msgs)

ament_package()