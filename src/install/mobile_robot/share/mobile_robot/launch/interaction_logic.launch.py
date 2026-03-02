#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    find_node = Node(
        package='mobile_robot',
        executable='identify5',
        name='identify5',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    move_node = Node(
        package='mobile_robot',
        executable='move5',
        name='move5',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    avoid_node = Node(
        package='mobile_robot',
        executable='avoid_while_interact',
        name='avoid_while_interact',
        output='screen',
        parameters=[{'use_sim_time': True}]
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

    return LaunchDescription([
        bridge_odom,
        find_node,
        move_node,
        avoid_node,
    ])