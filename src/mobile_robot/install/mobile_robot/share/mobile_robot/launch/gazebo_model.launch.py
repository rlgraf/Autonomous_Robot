###############################################################################
# ROS2 and Gazebo Launch File of the differential drive robot
# Author: Aleksander Haber
# This code is the ownership of Aleksander Haber
# Read the license!
###############################################################################

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
import xacro


def generate_launch_description():

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
    
    
    # this is the launch description
    
    # this is if you are using your own world model
    # gazeboLaunch=IncludeLaunchDescription(gazebo_rosPackageLaunch, launch_arguments={'gz_args': ['-r -v -v4 ', pathWorldFile], 'on_exit_shutdown': 'true'}.items())
    
    # this is if you are using an empty world model
    gazeboLaunch=IncludeLaunchDescription(gazebo_rosPackageLaunch,
                                          launch_arguments={'gz_args': ['-r -v -v4 empty.sdf'],
                                                            'on_exit_shutdown': 'true'}.items())                                                                   'launch', 'gz_sim.launch.py'))


    # Gazebo node
    spawnModelNodeGazebo = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', robotXacroname,
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

    # here we create an empty launch description object
    launchDescriptionObject = LaunchDescription()

    # we add gazeboLaunch
    launchDescriptionObject.add_action(gazeboLaunch)

    # we add the two nodes
    launchDescriptionObject.add_action(spawnNodeGazebo)
    launchDescriptionObject.add_action(nodeRobotStatePublisher)
    launchDescriptionObject.add_action(start_gazebo_ros_bridge_cmd)

    return launchDescriptionObject