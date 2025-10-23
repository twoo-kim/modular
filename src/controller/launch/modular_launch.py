from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
  # Directory for the package
  pkg_dir = get_package_share_directory('modular')
  
  # Path to configuration and model files
  config_path = os.path.join(pkg_dir, 'config', 'modular_params.yaml')

  # Get N
  with open(config_path, 'r') as f:
    config = yaml.safe_load(f)
    N = config['N']

  # Declare launch arguments
  pose_topic_arg = DeclareLaunchArgument('pose_topic', default_value='/sim/pose')

  # LaunchConfiguration
  args = [pose_topic_arg]
  pose_topic = LaunchConfiguration('pose_topic')

  # Modules
  for i in range(N):
    # Simulator Node
    ctrl_node = Node(
      package = 'modular',
      executable = 'ctrl_node',
      name = f'ctrl_node{i+1}',
      output = 'screen',
      parameters = [{'index': i+1}],
      # prefix = 'xterm -e gdb -ex run --args',
      remappings = [
        ('/pose_topic', pose_topic),
      ]
    )
    args.append(ctrl_node)

  # Simulator Node
  simulator_node = Node(
    package = 'modular',
    executable = 'sim_node',
    name = 'sim_node',
    output = 'screen',
    # prefix = 'xterm -e gdb -ex run --args',
    parameters = [
      {'config_path': config_path},
    ],
    remappings = [
      ('/pose_topic', pose_topic),
    ]
  )
  args.append(simulator_node)

  return LaunchDescription(args)