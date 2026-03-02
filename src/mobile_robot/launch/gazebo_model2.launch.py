#!/usr/bin/env python3
###############################################################################
# ROS2 and Gazebo Launch File of the differential drive robot
# Generates a random cylinder arena at launch time
###############################################################################
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    ExecuteProcess,
    RegisterEventHandler,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
import xacro


def generate_launch_description():

    robotXacroName = 'differential_drive_robot'
    namePackage    = 'mobile_robot'

    # ── Paths ──────────────────────────────────────────────────────────────────
    pathModelFile = os.path.join(
        get_package_share_directory(namePackage),
        'model', 'robot.xacro'
    )
    pathGenScript = os.path.join(
        get_package_share_directory(namePackage),
        'worlds', 'arena2.py'
    )
    bridge_params = os.path.join(
        get_package_share_directory(namePackage),
        'parameters', 'bridge_parameters.yaml'
    )

    generatedWorldPath = '/tmp/arena_generated.sdf'

    robotDescription = xacro.process_file(pathModelFile).toxml()

    # ── 1. Generate world SDF ──────────────────────────────────────────────────
    generateWorld = ExecuteProcess(
        cmd=['python3', pathGenScript, '--out', generatedWorldPath],
        output='screen'
    )

    # ── 2. Gazebo (only starts after generator exits successfully) ─────────────
    gazeboLaunch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args': ['-r -v4 ', generatedWorldPath],
            'on_exit_shutdown': 'true',
        }.items()
    )

    # ── 3. Robot state publisher ───────────────────────────────────────────────
    nodeRobotStatePublisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robotDescription,
            'use_sim_time': True,
        }]
    )

    # ── 4. Spawn robot ─────────────────────────────────────────────────────────
    spawnModelNodeGazebo = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', robotXacroName, 
            '-topic', 'robot_description',
            '-x', '-9.0',
            '-y', '4.0',
        ],
        output='screen'
    )

    # ── 5. ROS <-> Gazebo bridge ───────────────────────────────────────────────
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

    # ── 6. Static TF alias for raw Gazebo lidar frame ─────────────────────────
    lidar_frame_alias_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '0', '0', '0',
            '0', '0', '0',
            'lidar_link',
            'differential_drive_robot/base_footprint/gpu_lidar',
        ],
        output='screen'
    )

    # ── Launch description ─────────────────────────────────────────────────────
    ld = LaunchDescription()

    # Step 1: generate the world file
    ld.add_action(generateWorld)

    # Step 2: start Gazebo only if generator exited with code 0
    def on_generator_exit(event, context):
        if event.returncode != 0:
            print(
                f'\n[ERROR] World generator failed (exit code {event.returncode}). '
                'Check that gen_arena_world.py is installed — see setup.py data_files.\n'
            )
            return []
        return [gazeboLaunch]

    ld.add_action(
        RegisterEventHandler(
            OnProcessExit(
                target_action=generateWorld,
                on_exit=on_generator_exit,
            )
        )
    )

    # Steps 3-6: start immediately, will connect once Gazebo is ready
    ld.add_action(nodeRobotStatePublisher)
    ld.add_action(spawnModelNodeGazebo)
    ld.add_action(start_gazebo_ros_bridge_cmd)
    ld.add_action(lidar_frame_alias_tf)

    return ld