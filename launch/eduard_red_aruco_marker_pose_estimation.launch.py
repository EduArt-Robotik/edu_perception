import os

from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
  package_path = FindPackageShare('edu_perception')
  parameter_file = PathJoinSubstitution([
    package_path,
    'parameter',
    'eduard_red_aruco_marker_pose_estimation.yaml'
  ])
  robot_namespace = EnvironmentVariable('EDU_ROBOT_NAMESPACE', default_value="eduard")

  oak_camera_node = Node(
    package='edu_perception',
    executable='oak_d_camera',
    name='oak_d_camera',
    parameters=[parameter_file],
    namespace=EnvironmentVariable('EDU_ROBOT_NAMESPACE', default_value="eduard"),
    output='screen'
  )

  aruco_marker_detection_node = Node(
    package='aruco_opencv',
    executable='aruco_tracker_autostart',
    name='aruco_tracker',
    parameters=[parameter_file],
    remappings=[
      ('/oak_d_camera/camera_info', 'oak_d_camera/camera_info')
    ],
    namespace=EnvironmentVariable('EDU_ROBOT_NAMESPACE', default_value="eduard"),
    output='screen'
  )

  tf_publisher_cam_front = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    arguments=[
      '0.145', '0', '0.07', '1.570796327', '3.141592654', '1.570796327',
      PathJoinSubstitution([robot_namespace, 'base_link']),
      PathJoinSubstitution([robot_namespace, 'oak_d'])
    ]
  )

  return LaunchDescription([
    oak_camera_node,
    aruco_marker_detection_node,
    tf_publisher_cam_front
  ])