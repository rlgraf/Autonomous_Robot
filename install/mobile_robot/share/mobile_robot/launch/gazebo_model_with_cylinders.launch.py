import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    robotXacroName = 'differential_drive_robot'
    namePackage = 'mobile_robot'

    modelFileRelativePath = 'model/robot.xacro'
    pathModelFile = os.path.join(get_package_share_directory(namePackage), modelFileRelativePath)
    robotDescription = xacro.process_file(pathModelFile).toxml()

    gazebo_rosPackageLaunch = PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
    )

    # Generator script inside your package share
    genScriptRelPath = 'worlds/arena_moving_cylinder.py'
    pathGenScript = os.path.join(get_package_share_directory(namePackage), genScriptRelPath)

    # Output world in a writable location
    generatedWorldPath = os.path.join('/tmp', 'arena_50x50_random.world.sdf')

    generateWorld = ExecuteProcess(
        cmd=['python3', pathGenScript, '--out', generatedWorldPath],
        output='screen'
    )

    # Launch Gazebo using the generated world
    gazeboLaunch = IncludeLaunchDescription(
        gazebo_rosPackageLaunch,
        launch_arguments={
            'gz_args': ['-r -v -v4 ', generatedWorldPath],
            'on_exit_shutdown': 'true'
        }.items()
    )

    spawnModelNodeGazebo = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', robotXacroName, '-topic', 'robot_description'],
        output='screen'
    )

    nodeRobotStatePublisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robotDescription, 'use_sim_time': True}]
    )

    bridge_params = os.path.join(
        get_package_share_directory(namePackage),
        'parameters',
        'bridge_parameters.yaml'
    )

    start_gazebo_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', f'config_file:={bridge_params}'],
        output='screen',
    )

    ld = LaunchDescription()

    # 1) generate the world first
    ld.add_action(generateWorld)

    # 2) start Gazebo only after generator finishes
    ld.add_action(
        RegisterEventHandler(
            OnProcessExit(
                target_action=generateWorld,
                on_exit=[gazeboLaunch],
            )
        )
    )

    # 3) bring up robot + bridge (these can start now; they’ll connect once sim is up)
    ld.add_action(nodeRobotStatePublisher)
    ld.add_action(spawnModelNodeGazebo)
    ld.add_action(start_gazebo_ros_bridge_cmd)

    return ld
