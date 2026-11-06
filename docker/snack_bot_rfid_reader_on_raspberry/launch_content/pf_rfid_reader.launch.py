import os

from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, EnvironmentVariable, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
  # robot namespace
  edu_robot_namespace = LaunchConfiguration('edu_robot_namespace')
  edu_robot_namespace_arg = DeclareLaunchArgument(
      'edu_robot_namespace', default_value=os.getenv('EDU_ROBOT_NAMESPACE', default='eduard')
  )

  # RFID reader node
  rfid_reader_parameter = PathJoinSubstitution([
    './',
    'pf_rfid_reader.yaml'
  ])

  rfid_reader_node = Node(
    package='edu_perception',
    executable='pf_rfid_reader',
    name='pf_rfid_reader',
    namespace=edu_robot_namespace,
    parameters=[rfid_reader_parameter]
  )

  return LaunchDescription([
      edu_robot_namespace_arg,
      rfid_reader_node
  ])
