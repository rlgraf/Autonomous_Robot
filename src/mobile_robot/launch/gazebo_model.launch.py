###############################################################################
# ROS2 and Gazebo Launch File of the differential drive robot
###############################################################################

import os
import yaml
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    TimerAction,
    SetEnvironmentVariable,
    ExecuteProcess,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():
    from launch.actions import DeclareLaunchArgument
    from launch.substitutions import LaunchConfiguration

    # Declare launch argument for real-time factor
    declare_real_time_factor = DeclareLaunchArgument(
        'real_time_factor',
        default_value='1.0',
        description='Simulation speed multiplier (1.0 = normal, 2.0 = 2x speed, etc.)'
    )

    # Declare launch arguments for robot spawn position
    declare_robot_x = DeclareLaunchArgument(
        'robot_x',
        default_value='-20.0',
        description='Robot spawn X coordinate (meters)'
    )
    declare_robot_y = DeclareLaunchArgument(
        'robot_y',
        default_value='-1.5',
        description='Robot spawn Y coordinate (meters)'
    )
    declare_robot_z = DeclareLaunchArgument(
        'robot_z',
        default_value='0.0',
        description='Robot spawn Z coordinate (meters)'
    )

    # Declare launch argument for coordinates file
    declare_coordinates_file = DeclareLaunchArgument(
        'coordinates_file',
        default_value='',
        description='Path to file with cylinder coordinates (one per line: x,y)'
    )


    robotXacroName = 'differential_drive_robot'
    namePackage = 'mobile_robot'

    pkg_share = get_package_share_directory(namePackage)

    pathModelFile = os.path.join(pkg_share, 'model', 'robot.xacro')

    # IMPORTANT:
    # This must match where arena2.py is installed in your package share.
    # Change 'worlds' to 'world' if that is your actual installed folder name.
    pathGenScript = os.path.join(pkg_share, 'world', 'arena2.py')
    pathGenWrapper = os.path.join(pkg_share, 'world', 'generate_world_wrapper.py')

    generatedWorldPath = '/tmp/arena_generated.sdf'

    print("Generator script:", pathGenScript, "exists:", os.path.exists(pathGenScript))
    print("Generated world target:", generatedWorldPath)

    # Robot description from xacro
    robotDescription = xacro.process_file(pathModelFile).toxml()

    # Gazebo launch source
    gazebo_rosPackageLaunch = PythonLaunchDescriptionSource(
        os.path.join(
            get_package_share_directory('ros_gz_sim'),
            'launch',
            'gz_sim.launch.py'
        )
    )

    # 1. Generate the random world first
    real_time_factor = LaunchConfiguration('real_time_factor')
    coordinates_file = LaunchConfiguration('coordinates_file')
    
    # Build command for world generation
    world_gen_cmd = ['python3', pathGenScript, '--out', generatedWorldPath, 
                     '--real-time-factor', real_time_factor]
    
    # Add coordinates file if provided
    from launch.conditions import IfCondition
    from launch.substitutions import PythonExpression
    
    # Use wrapper script to handle optional coordinates_file argument
    # The wrapper will skip --coordinates-file if the value is empty
    generateWorld = ExecuteProcess(
        cmd=['python3', pathGenWrapper, '--out', generatedWorldPath, 
             '--real-time-factor', real_time_factor,
             '--coordinates-file', coordinates_file],
        output='screen'
    )

    # 2. Launch Gazebo using the generated world
    # To speed up simulation, add --real-time-factor 2.0 (or desired multiplier) to gz_args
    # Example: 'gz_args': f'-r -v4 --real-time-factor 2.0 {generatedWorldPath}'
    gazeboLaunch = IncludeLaunchDescription(
        gazebo_rosPackageLaunch,
        launch_arguments={
            'gz_args': f'-r -v4 {generatedWorldPath}',
            'on_exit_shutdown': 'true'
        }.items()
    )

    # Spawn robot
    robot_x = LaunchConfiguration('robot_x')
    robot_y = LaunchConfiguration('robot_y')
    robot_z = LaunchConfiguration('robot_z')
    spawnModelNodeGazebo = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', robotXacroName,
            '-topic', 'robot_description',
            '-x', robot_x,
            '-y', robot_y,
            '-z', robot_z,
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

    # Shared simulation parameters file
    shared_params_file = os.path.join(
        pkg_share,
        'parameters',
        'battery_tunable_parameters.yaml'
    )

    with open(shared_params_file, 'r') as f:
        shared_params = yaml.safe_load(f)

    battery_ros_params = shared_params['battery_node']['ros__parameters']

    charging_x = list(battery_ros_params.get('charging_station_x', []))
    charging_y = list(battery_ros_params.get('charging_station_y', []))

    if len(charging_x) != len(charging_y):
        raise ValueError(
            'charging_station_x and charging_station_y must have the same length'
        )

    charging_stations = [
        (float(x), float(y)) for x, y in zip(charging_x, charging_y)
    ]

    charging_radius = float(
        battery_ros_params.get('charging_radius', 0.5)
    )

    # Battery node
    start_battery_cmd = Node(
        package='mobile_robot',
        executable='battery_node',
        name='battery_node',
        output='screen',
        parameters=[shared_params_file],
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
                    '-x', str(sx),
                    '-y', str(sy),
                    '-z', '0.01',
                ],
                output='screen'
            )
        )

    # Ensure Gazebo can resolve model://person URIs
    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.join(pkg_share, 'model')
    )

    ld = LaunchDescription()

    ld.add_action(declare_real_time_factor)
    ld.add_action(declare_robot_x)
    ld.add_action(declare_robot_y)
    ld.add_action(declare_robot_z)
    ld.add_action(declare_coordinates_file)
    ld.add_action(set_gz_resource_path)
    ld.add_action(nodeRobotStatePublisher)
    ld.add_action(generateWorld)

    def on_generator_exit(event, context):
        if event.returncode != 0:
            print(
                f'\n[ERROR] World generator failed with exit code {event.returncode}\n'
                f'Generator path: {pathGenScript}\n'
                f'Check whether arena2.py is installed in the package share.\n'
            )
            return []

        return [
            gazeboLaunch,
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
                actions=spawn_station_nodes
            ),
        ]

    ld.add_action(
        RegisterEventHandler(
            OnProcessExit(
                target_action=generateWorld,
                on_exit=on_generator_exit
            )
        )
    )

    return ld