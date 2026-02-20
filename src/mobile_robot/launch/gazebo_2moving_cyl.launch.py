#!/usr/bin/env python3
###############################################################################
# ROS2 + Gazebo Sim launch: moving-cylinder world + differential drive robot
# - Launches Gazebo with arena_moving_cylinder.sdf
# - Publishes robot_description from xacro via robot_state_publisher
# - Spawns robot into Gazebo after a delay
# - Starts ros_gz_bridge using bridge_parameters.yaml
# - Adds a TF alias for the bridged LiDAR scan frame_id
# - Launches bounce_cylinder.py as a ROS 2 node to drive the moving cylinder
#
# IMPORTANT:
# - Forces Gazebo + ros_gz_bridge into the same Gazebo Transport space via GZ_PARTITION
###############################################################################

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    # ------------------------- User config -------------------------
    namePackage = 'mobile_robot'
    robotXacroName = 'differential_drive_robot'

    modelFileRelativePath = 'model/robot.xacro'
    worldFileRelativePath = 'worlds/arena_2moving_cylinders.sdf'

    # bounce_cylinder.py should be installed into your package (recommended: scripts/)
    # Example install: install(PROGRAMS scripts/bounce_cylinder.py DESTINATION lib/${PROJECT_NAME})
    bounceNodeExecutable = 'bounce_cylinder.py'  # executable name after install

    # Spawn pose (tweak as needed)
    spawn_x = '0.0'
    spawn_y = '0.0'
    spawn_z = '0.25'
    spawn_yaw = '0.0'

    # Delay spawn / script start to let Gazebo initialize
    spawn_delay_sec = 4.0

    # ---- Gazebo transport isolation fix ----
    gz_partition = 'arena_partition'
    # ---------------------------------------------------------------

    # Absolute paths
    pkg_share = get_package_share_directory(namePackage)
    pathModelFile = os.path.join(pkg_share, modelFileRelativePath)
    pathWorldFile = os.path.join(pkg_share, worldFileRelativePath)

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

    # robot_state_publisher
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

    delayed_spawn = TimerAction(
        period=spawn_delay_sec,
        actions=[spawnModelNodeGazebo]
    )

    # Bridge config file
    bridge_params = os.path.join(pkg_share, 'parameters', 'bridge_parameters.yaml')

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
        ],
    )

    # EKF (path fixed)
    ekf_yaml = os.path.join(pkg_share, 'parameters', 'ekf_global.yaml')
    ekf_global = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_global',
        output='screen',
        parameters=[ekf_yaml],
    )

    # TF alias for LaserScan frame_id from bridge -> real URDF frame
    lidar_frame_alias_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=[
            '0', '0', '0',
            '0', '0', '0',
            'lidar_link',
            'differential_drive_robot/base_footprint/gpu_lidar',
        ],
    )

    # Bounce cylinder ROS node (publishes Twist to /model/cylinder_moving/cmd_vel)
    bounceCylinderNode = Node(
        package='mobile_robot',
        executable='bounce_2cylinders',
        name='bounce_2cylinders',
        output='screen',
    )


    delayed_bounce = TimerAction(
        period=spawn_delay_sec,
        actions=[bounceCylinderNode]
    )

    ld = LaunchDescription()
    ld.add_action(SetEnvironmentVariable('GZ_PARTITION', gz_partition))
    ld.add_action(gazeboLaunch)
    ld.add_action(nodeRobotStatePublisher)
    ld.add_action(start_gazebo_ros_bridge_cmd)
    ld.add_action(lidar_frame_alias_tf)
    ld.add_action(delayed_spawn)
    ld.add_action(delayed_bounce)
    ld.add_action(ekf_global)

    return ld
