###############################################################################
# ROS2 and Gazebo Launch File of the differential drive robot
# Author: Aleksander Haber
# This code is the ownership of Aleksander Haber
# Read the license!
###############################################################################

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, SetEnvironmentVariable
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
    pathModelFile = os.path.join(get_package_share_directory(namePackage),
                                 modelFileRelativePath)

    # uncomment this if you ar using your own world model
    # this is the absolute path to the world model
    # pathWorldFile = os.path.join(get_package_share_directory(namePackage), worldFileRelativePath)

    # get the robot description from the xacro model file
    robotDescription = xacro.process_file(pathModelFile).toxml()


    # this is the launch file from the gazebo_ros package
    gazebo_rosPackageLaunch=PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ros_gz_sim'),
                                                                        'launch', 'gz_sim.launch.py'))
    
    
    # this is the launch description
    
    # this is if you are using your own world model
    # gazeboLaunch=IncludeLaunchDescription(gazebo_rosPackageLaunch, launch_arguments={'gz_args': ['-r -v -v4 ', pathWorldFile], 'on_exit_shutdown': 'true'}.items())
    
    # this is if you are using an empty world model
    # gazeboLaunch=IncludeLaunchDescription(gazebo_rosPackageLaunch,
    #                                       launch_arguments={'gz_args': '-r empty.sdf',
    #                                                         'on_exit_shutdown': 'true'}.items())                                                
    # AFTER
    gazeboLaunch=IncludeLaunchDescription(gazebo_rosPackageLaunch,
                                          launch_arguments={
                                            'gz_args': '-r -v -v4 default.sdf',
                                            'on_exit_shutdown': 'true'
                                        }.items())

    # Gazebo node
    spawnModelNodeGazebo = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', robotXacroName,
            '-topic', 'robot_description'
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
        '-p',
        f'config_file:={bridge_params}',
    ],
    output='screen',
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

    start_battery_cmd = Node(
        package='mobile_robot',
        executable='battery_node',
        name='battery_node',
        output='screen',
        parameters=[params_battery_file],   # loads everything automatically
    )

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

    # ───────────────────────────────────────────────────────────────────────
    # PEOPLE (spawn mesh-based "person" model instances)
    # Requires:
    #   model/person/model.config
    #   model/person/model.sdf
    #   model/person/meshes/scene.gltf + scene.bin
    # and setup.py installs these into share/mobile_robot/model/...
    # ───────────────────────────────────────────────────────────────────────

    # Simple list of people positions (x, y) in world coordinates
    people_positions = [
        (1.0, 1.0),
        (3.0, 1.0),
        (3.0, 3.0),
    ]

    person_model_sdf_path = os.path.join(
        get_package_share_directory(namePackage),
        'model', 'person', 'model.sdf'
    )

    spawn_people_nodes = []
    for j, (px, py) in enumerate(people_positions):
        spawn_people_nodes.append(
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-file', person_model_sdf_path,
                    '-name', f'person_{j}',
                    '-x', str(px),
                    '-y', str(py),
                    '-z', '0.0',
                ],
                output='screen'
            )
        )

    # ------------------------------------------------------------------ #
    # Launch description                                                   #
    # Spawn is delayed 5s to give Gazebo time to fully initialize         #
    # ------------------------------------------------------------------ #
    return LaunchDescription([
        # Make Gazebo able to resolve model://person/... (and other local models)
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=os.path.join(get_package_share_directory(namePackage), 'model')
        ),

        # Start Gazebo and robot state publisher immediately
        gazeboLaunch,
        nodeRobotStatePublisher,

        # Wait 5 seconds then spawn the robot
        TimerAction(
            period=5.0,
            actions=[spawnModelNodeGazebo]
        ),
        
        # Wait 6 seconds then start the bridge (after spawn)
        TimerAction(
            period=6.0,
            actions=[start_gazebo_ros_bridge_cmd]
        ),

        # Battery node (may still crash if its code/YAML has an issue)
        TimerAction(
            period=7.0,
            actions=[start_battery_cmd]
        ),

        # Spawn recharge stations
        TimerAction(
            period=8.0,
            actions=[*spawn_station_nodes]
        ),

        # Spawn people
        TimerAction(
            period=9.0,
            actions=[*spawn_people_nodes]
        ),
    ])