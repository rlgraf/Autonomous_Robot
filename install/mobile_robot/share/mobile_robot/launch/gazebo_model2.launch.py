#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description():

    pkg_share = get_package_share_directory('mobile_robot')

    shared_params_file = os.path.join(
        pkg_share,
        'parameters',
        'battery_tunable_parameters.yaml'
    )

    decider_params = os.path.join(
        pkg_share,
        'parameters',
        'decider_parameters.yaml'
    )

    # Bridge existing Gazebo odometry from the already-running arena
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
    
    # Supervisor should receive the shared params too, especially if it now
    # contains battery-aware decision thresholds and charging station info.
    supervisor_node = Node(
        package='mobile_robot',
        executable='supervisor_node',
        name='supervisor_node',
        output='screen',
        parameters=[decider_params, {'use_sim_time': True}],
    )

    
    # Recharge behavior should come up early on restart so it can immediately
    # react to the current /battery_status being published by battery_node
    # from gazebo_model.launch.py.
    auto_recharge_node = Node(
        package='mobile_robot',
        executable='auto_recharge_node',
        name='auto_recharge_node',
        output='screen',
        parameters=[shared_params_file, {'use_sim_time': True}],
    )
    
    data_logger_node = Node(
        package='mobile_robot',
        executable='data_logger',
        name='data_logger',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )
    soft_avoidance_node = Node(
        package='mobile_robot',
        executable='soft_obstacle_avoidance_node',
        name='soft_obstacle_avoidance_node',
        output='screen',
        parameters=[shared_params_file, {'use_sim_time': True}],
        remappings=[
            ('/cmd_vel', '/cmd_vel_soft_avoid'),
        ],
    )

    find_node = Node(
        package='mobile_robot',
        executable='identify5',
        name='identify5',
        output='screen',
        parameters=[shared_params_file, {'use_sim_time': True}],
    )

    move_node = Node(
        package='mobile_robot',
        executable='move5',
        name='move5',
        output='screen',
        parameters=[shared_params_file, {'use_sim_time': True}],
        remappings=[
            ('/cmd_vel', '/cmd_vel_nav'),
        ],
    )

    avoid_node = Node(
        package='mobile_robot',
        executable='avoid_while_interact',
        name='avoid_while_interact',
        output='screen',
        parameters=[shared_params_file, {'use_sim_time': True}],
        remappings=[
            ('/cmd_vel', '/cmd_vel_avoid'),
        ],
    )

    return LaunchDescription([
        bridge_odom,

        TimerAction(period=0.4, actions=[supervisor_node]),
        TimerAction(period=1.0, actions=[auto_recharge_node]),
        TimerAction(period=1.4, actions=[soft_avoidance_node]),
        TimerAction(period=2.0, actions=[find_node]),
        TimerAction(period=2.4, actions=[move_node]),
        TimerAction(period=2.8, actions=[avoid_node]),
        TimerAction(period=3.2, actions=[data_logger_node]),
    ])