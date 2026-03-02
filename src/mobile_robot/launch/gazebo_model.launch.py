###############################################################################
# ROS2 and Gazebo Launch File of the differential drive robot
###############################################################################

import os
import yaml
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

def generate_launch_description():

    robotXacroName = 'differential_drive_robot'
    namePackage = 'mobile_robot'

    pkg_share = get_package_share_directory(namePackage)
    modelFileRelativePath = 'model/robot.xacro'
    pathModelFile = os.path.join(pkg_share, modelFileRelativePath)
    
    # Robot description from xacro
    robotDescription = xacro.process_file(pathModelFile).toxml()

    # Gazebo launch (DEFAULT world)
    gazebo_rosPackageLaunch = PythonLaunchDescriptionSource(
        os.path.join(
            get_package_share_directory('ros_gz_sim'),
            'launch',
            'gz_sim.launch.py'
        )  
    )

    arena2_sdf = "/home/corelab/Autonomous_Robot/src/mobile_robot/world/arena2.sdf"
    print("Loading world file:", arena2_sdf, "exists:", os.path.exists(arena2_sdf))

    gazeboLaunch = IncludeLaunchDescription(
        gazebo_rosPackageLaunch,
        launch_arguments={
            'gz_args': f'-r {arena2_sdf}',
            'on_exit_shutdown': 'true'
        }.items()
    )
   

    # Spawn robot
    spawnModelNodeGazebo = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', robotXacroName,
            '-topic', 'robot_description',
            '-x', '-9.0',
            '-y', '0.0',
            '-z', '0.0',
        ],
        output='screen'
    )

    # Robot State Publisher
    nodeRobotStatePublisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robotDescription,
            'use_sim_time': True
        }]
    )

    # Bridge
    bridge_params = os.path.join(
        pkg_share,
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

    # Battery parameters
    params_battery_file = os.path.join(
        pkg_share,
        'parameters',
        'battery_tunable_parameters.yaml'
    )

    with open(params_battery_file, 'r') as f:
        battery_params = yaml.safe_load(f)

    ros_params = battery_params['battery_node']['ros__parameters']

    charging_stations = [
        tuple(s) for s in ros_params.get('charging_stations', [])
    ]

    charging_radius = float(
        ros_params.get('charging_radius', 0.5)
    )

    # Battery node
    start_battery_cmd = Node(
        package='mobile_robot',
        executable='battery_node',
        name='battery_node',
        output='screen',
        parameters=[params_battery_file],
    )

    # Charging station template
    station_sdf_template_path = os.path.join(
        pkg_share,
        'model',
        'charging_stations.sdf'
    )

    with open(station_sdf_template_path, 'r') as f:
        station_sdf_template = f.read()

    spawn_station_nodes = []

    for i, (sx, sy) in enumerate(charging_stations):

        sdf_string = (
            station_sdf_template
            .replace('STATION_ID', str(i))
            .replace('STATION_RADIUS', str(charging_radius))
        )

        spawn_station_nodes.append(
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-string', sdf_string,
                    '-name', f'charging_station_{i}',
                    '-x', str(float(sx)),
                    '-y', str(float(sy)),
                    '-z', '0.01',
                ],
                output='screen'
            )
        )

    # Person model spawning
    person_model_sdf_path = os.path.join(
        pkg_share,
        'model',
        'person',
        'model.sdf'
    )

    # Ensure Gazebo can resolve model://person URIs
    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.join(pkg_share, 'model')
    )

    return LaunchDescription([

        set_gz_resource_path,

        gazeboLaunch,
        nodeRobotStatePublisher,

        TimerAction(
            period=4.0,
            actions=[spawnModelNodeGazebo]
        ),

        TimerAction(
            period=4.5,
            actions=[start_gazebo_ros_bridge_cmd]
        ),

        TimerAction(
            period=5.0,
            actions=[start_battery_cmd]
        ),

        TimerAction(
            period=5.5,
            actions=[*spawn_station_nodes]
        ),
    ])