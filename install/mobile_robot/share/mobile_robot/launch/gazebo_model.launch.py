###############################################################################
# ROS2 and Gazebo Launch File of the differential drive robot
# Author: Aleksander Haber
# This code is the ownership of Aleksander Haber
# Read the license!
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

from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch_ros.actions import Node
import yaml
import xacro


def generate_launch_description():
    # set_qt_platform = SetEnvironmentVariable('QT_QPA_PLATFORM', 'xcb')
    # set_render_engine = SetEnvironmentVariable('GZ_RENDER_ENGINE_NAME', 'ogre')

    # this name has to match the robor name in the Xacro file
    robotXacroName='differential_drive_robot'

    # this is the name of our package, at the same time this is the name of the 
    # folder that will be used to define the paths
    namePackage = 'mobile_robot'

    # this is a relative path to the xacro file defining the model
    modelFileRelativePath = 'model/robot.xacro'

    # uncomment this if you want to define your own empty world model
    # however, then you have to create empty_world.world
    # this is a relative path to the Gazebo world file
    # worldFileRelativePath = 'model/empty_world.world'

    # this is the absolute path to the model
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

    # uncomment this if you ar using your own world model
    # this is the absolute path to the world model
    # pathWorldFile = os.path.join(get_package_share_directory(namePackage), worldFileRelativePath)

    # get the robot description from the xacro model file
    # robotDescription = xacro.process_file(pathModelFile).toxml()


    # # this is the launch file from the gazebo_ros package
    # gazebo_rosPackageLaunch=PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ros_gz_sim'),
    #                                                                     'launch', 'gz_sim.launch.py'))
    
    
    # # this is the launch description
    
    # # this is if you are using your own world model
    # # gazeboLaunch=IncludeLaunchDescription(gazebo_rosPackageLaunch, launch_arguments={'gz_args': ['-r -v -v4 ', pathWorldFile], 'on_exit_shutdown': 'true'}.items())
    
    # # this is if you are using an empty world model
    # # gazeboLaunch=IncludeLaunchDescription(gazebo_rosPackageLaunch,
    # #                                       launch_arguments={'gz_args': '-r empty.sdf',
    # #                                                         'on_exit_shutdown': 'true'}.items())                                                
    # # AFTER
    # gazeboLaunch=IncludeLaunchDescription(gazebo_rosPackageLaunch,
    #                                       launch_arguments={
    #                                         'gz_args': '-r -v -v4 default.sdf',
    #                                         'on_exit_shutdown': 'true'
    #                                     }.items())
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


    # Gazebo node
    spawnModelNodeGazebo = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', robotXacroName, 
            '-topic', 'robot_description',
            '-x', '-49.0',
            '-y', '4.0',
        ],
        output='screen'
    )


    # Robot State Publisher Node
    nodeRobotStatePublisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robotDescription,
        'use_sim_time': True}]
    )

    # this is very important so we can control the robot from ROS2
    bridge_params = os.path.join(
    get_package_share_directory(namePackage),
    'parameters',
    'bridge_parameters.yaml'
    )

    start_gazebo_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
        '--ros-args',
        '-p', f'config_file:={bridge_params}',
        '-p', 'use_sim_time:=True',
        '-p', 'override_frame_id:=lidar_link',
    ],
    output='screen',
    )

    bridge_odom = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='odom_gt_bridge',
        arguments=[
            '/model/differential_drive_robot/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry'
        ],
        remappings=[
            ('/model/differential_drive_robot/odometry', '/odom_gt')
        ],
        output='screen'
    )


    params_battery_file = os.path.join(
        get_package_share_directory(namePackage),
        'parameters', 'battery_tunable_parameters.yaml'
    )
    with open(params_battery_file, 'r') as f:
        battery_params = yaml.safe_load(f)
    charging_stations = [
        tuple(s) for s in battery_params['battery_node']['ros__parameters']['charging_stations']
    ]


    # radius comes straight from the YAML
    charging_radius = battery_params['battery_node']['ros__parameters']['charging_radius']

        # ── Load charging-station SDF template from model file ─────────────── #
    station_sdf_template_path = os.path.join(
        get_package_share_directory(namePackage),
        'model', 'charging_stations.sdf'
    )
    with open(station_sdf_template_path, 'r') as f:
        station_sdf_template = f.read()

    spawn_station_nodes = []
    for i, (sx, sy) in enumerate(charging_stations):
        sdf_string = (station_sdf_template
                      .replace('STATION_ID',    str(i))
                      .replace('STATION_RADIUS',str(charging_radius)))


        spawn_station_nodes.append(
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-string', sdf_string,
                    '-name', f'charging_station_{i}',
                    '-x', str(sx),
                    '-y', str(sy),
                    '-z', '0.01',
                ],
                output='screen'
            )
            
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
    

        # ------------------------------------------------------------------ #
    # Launch description                                                   #
    # Spawn is delayed 5s to give Gazebo time to fully initialize         #
    # ------------------------------------------------------------------ #
    # return LaunchDescription([
    #     # Environment must be set before Gazebo launches
    #     # SetEnvironmentVariable('QT_QPA_PLATFORM', 'xcb'),
        
    #     # Start Gazebo and robot state publisher immediately
    #     gazeboLaunch,
    #     nodeRobotStatePublisher,

    #     # Wait 5 seconds then spawn the robot
    #     TimerAction(
    #         period=5.0,           
    #         actions=[spawnModelNodeGazebo]
    #     ),
        
    #     # Wait 6 seconds then start the bridge (after spawn)
    #     TimerAction(
    #         period=6.0,
    #         actions=[start_gazebo_ros_bridge_cmd]
    #     ),
    #     TimerAction(
    #         period=6.0,
    #         actions=[bridge_odom]
    #     ),
    #     # TimerAction(
    #     #     period=7.0,   # just after bridge at 15s
    #     #     actions=[start_battery_cmd]
    #     # ),
    #     TimerAction(
    #         period=8.0,  # stagger spawn of recharge stations
    #         actions=[*spawn_station_nodes]
    #     ),
    # ])
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
    ld.add_action(bridge_odom)
    for node in spawn_station_nodes:
        ld.add_action(node)


    return ld