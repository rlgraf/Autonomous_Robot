#!/usr/bin/env python3
###############################################################################
# ROS2 and Gazebo Launch File of the differential drive robot
# Author: Aleksander Haber
# (Rewritten to add a TF alias frame for the bridged LiDAR scan frame_id)
###############################################################################

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro


def generate_launch_description():

    # Must match robot name in Xacro
    robotXacroName = 'differential_drive_robot'

    # Package name
    namePackage = 'mobile_robot'

    # Relative paths
    modelFileRelativePath = 'model/robot.xacro'
    worldFileRelativePath = 'worlds/arena.sdf'

    # Absolute paths
    pathModelFile = os.path.join(
        get_package_share_directory(namePackage),
        modelFileRelativePath
    )
    pathWorldFile = os.path.join(
        get_package_share_directory(namePackage),
        worldFileRelativePath
    )

    # Robot description from xacro
    robotDescription = xacro.process_file(pathModelFile).toxml()

    # Gazebo Sim launch
    gz_sim_launch = PythonLaunchDescriptionSource(
        os.path.join(
            get_package_share_directory('ros_gz_sim'),
            'launch',
            'gz_sim.launch.py'
        )
    )

    gazeboLaunch = IncludeLaunchDescription(
        gz_sim_launch,
        launch_arguments={
            'gz_args': ['-r -v -v4 ', pathWorldFile],
            'on_exit_shutdown': 'true'
        }.items()
    )

    # Spawn robot into Gazebo
    spawnModelNodeGazebo = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', robotXacroName,
            '-topic', 'robot_description',
        ],
        output='screen'
    )

    # Publish TF from URDF
    nodeRobotStatePublisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robotDescription,
            'use_sim_time': True,
        }]
    )

    # Bridge config file
    bridge_params = os.path.join(
        get_package_share_directory(namePackage),
        'parameters',
        'bridge_parameters.yaml'
    )

    # Gazebo <-> ROS bridge
    # NOTE: Some ros_gz_bridge builds do NOT support override_frame_id,
    # but leaving it here is harmless. We additionally publish a TF alias below.
    start_gazebo_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        arguments=[
            '--ros-args',
            '-p', f'config_file:={bridge_params}',
            '-p', 'use_sim_time:=True',
            '-p', 'override_frame_id:=lidar_link',
        ],
        output='screen'
    )

    # TF alias for the LaserScan frame_id coming from ros_gz_bridge:
    # /scan.header.frame_id == "differential_drive_robot/base_footprint/gpu_lidar"
    # Make that frame exist by tying it to the real URDF lidar frame "lidar_link".
    #
    # This fixes TF lookup errors without needing to change ros_gz_bridge internals.
    lidar_frame_alias_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '0', '0', '0',     # x y z
            '0', '0', '0',     # roll pitch yaw
            'lidar_link',      # parent (real URDF frame)
            'differential_drive_robot/base_footprint/gpu_lidar',  # child (scan frame_id)
        ],
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(gazeboLaunch)
    ld.add_action(spawnModelNodeGazebo)
    ld.add_action(nodeRobotStatePublisher)
    ld.add_action(start_gazebo_ros_bridge_cmd)
    ld.add_action(lidar_frame_alias_tf)

    return ld
