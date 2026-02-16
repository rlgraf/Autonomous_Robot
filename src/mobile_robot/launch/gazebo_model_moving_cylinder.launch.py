#!/usr/bin/env python3
###############################################################################
# ROS2 + Gazebo Sim launch: moving-cylinder world + differential drive robot
# - Launches Gazebo with arena_moving_cylinder.sdf
# - Publishes robot_description from xacro via robot_state_publisher
# - Spawns robot into Gazebo with a safe Z offset (so it’s visible)
# - Starts ros_gz_bridge using your bridge_parameters.yaml
# - Adds a TF alias for the bridged LiDAR scan frame_id (optional, kept from your file)
###############################################################################

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro


def generate_launch_description():

    # ------------------------- User config -------------------------
    namePackage = 'mobile_robot'
    robotXacroName = 'differential_drive_robot'

    modelFileRelativePath = 'model/robot.xacro'
    worldFileRelativePath = 'worlds/arena_moving_cylinder.sdf'

    # Spawn pose (tweak as needed)
    spawn_x = '0.0'
    spawn_y = '0.0'
    spawn_z = '0.25'   # IMPORTANT: lift robot above ground so it’s visible
    spawn_yaw = '0.0'

    # Delay spawn to let Gazebo initialize
    spawn_delay_sec = 4.0
    # ---------------------------------------------------------------

    # Absolute paths
    pathModelFile = os.path.join(
        get_package_share_directory(namePackage),
        modelFileRelativePath
    )
    pathWorldFile = os.path.join(
        get_package_share_directory(namePackage),
        worldFileRelativePath
    )

    # Build robot_description from xacro
    robotDescription = xacro.process_file(pathModelFile).toxml()

    # Gazebo Sim launch include
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
            'gz_args': ['-r -v4 ', pathWorldFile],
            'on_exit_shutdown': 'true'
        }.items()
    )

    # robot_state_publisher (publishes /robot_description and TF)
    nodeRobotStatePublisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robotDescription,
            'use_sim_time': True,
        }]
    )

    # Spawn robot into Gazebo from /robot_description
    spawnModelNodeGazebo = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', robotXacroName,
            '-topic', 'robot_description',
            '-x', spawn_x,
            '-y', spawn_y,
            '-z', spawn_z,
            '-Y', spawn_yaw,
        ],
    )

    # Bridge config file
    bridge_params = os.path.join(
        get_package_share_directory(namePackage),
        'parameters',
        'bridge_parameters.yaml'
    )

    # Gazebo <-> ROS bridge
    start_gazebo_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        arguments=[
            '--ros-args',
            '-p', f'config_file:={bridge_params}',
            '-p', 'use_sim_time:=True',
            # Some builds ignore this; harmless to keep
            '-p', 'override_frame_id:=lidar_link',
        ],
    )

    # Optional: TF alias for LaserScan frame_id from bridge -> real URDF frame
    lidar_frame_alias_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=[
            '0', '0', '0',     # x y z
            '0', '0', '0',     # roll pitch yaw
            'lidar_link',      # parent (real URDF frame)
            'differential_drive_robot/base_footprint/gpu_lidar',  # child (scan frame_id)
        ],
    )

    # Delay spawning so Gazebo + robot_description are ready
    delayed_spawn = TimerAction(
        period=spawn_delay_sec,
        actions=[spawnModelNodeGazebo]
    )

    ld = LaunchDescription()
    ld.add_action(gazeboLaunch)
    ld.add_action(nodeRobotStatePublisher)
    ld.add_action(start_gazebo_ros_bridge_cmd)
    ld.add_action(lidar_frame_alias_tf)
    ld.add_action(delayed_spawn)

    return ld
