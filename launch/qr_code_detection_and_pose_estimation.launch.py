import os

from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    package_path = FindPackageShare('edu_perception')
    parameter_file = PathJoinSubstitution([
      package_path,
      'parameter',
      'qr_code_detection_and_pose_estimation.yaml'
    ])

    qr_detection_and_pose_estimation_node = Node(
      package='edu_perception',
      executable='qr_detection_and_pose_estimation',
      name='qr_detection_and_pose_estimation',
      parameters=[parameter_file],
      # prefix=['gdbserver localhost:3000'],
      output='screen'
    )

    return LaunchDescription([
      qr_detection_and_pose_estimation_node
    ])
    