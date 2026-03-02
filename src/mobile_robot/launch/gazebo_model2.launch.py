#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description():

    # -------------------------------
    # Bridge: Gazebo -> ROS2 odom_gt
    # -------------------------------
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

    # -------------------------------
    # Supervisor (arbiter) MUST start early
    # -------------------------------
    supervisor_node = Node(
        package='mobile_robot',
        executable='supervisor_node',
        name='supervisor_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # -------------------------------
    # Guest interaction stack (publish candidates)
    # -------------------------------
    find_node = Node(
        package='mobile_robot',
        executable='identify5',
        name='identify5',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    move_node = Node(
        package='mobile_robot',
        executable='move5',
        name='move5',
        output='screen',
        parameters=[{'use_sim_time': True}],
        remappings=[
            # move5 must publish candidate nav commands
            ('/cmd_vel', '/cmd_vel_nav'),
        ],
    )

    avoid_node = Node(
        package='mobile_robot',
        executable='avoid_while_interact',
        name='avoid_while_interact',
        output='screen',
        parameters=[{'use_sim_time': True}],
        remappings=[
            # avoid must publish candidate avoid commands
            ('/cmd_vel', '/cmd_vel_avoid'),
        ],
    )

    # -------------------------------
    # Battery + recharge stack (publish candidates + gates)
    # -------------------------------
    battery_node = Node(
        package='mobile_robot',
        executable='battery_node',
        name='battery_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    auto_recharge_node = Node(
        package='mobile_robot',
        executable='auto_recharge_node',
        name='auto_recharge_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
        remappings=[
            # autorecharge publishes a candidate velocity command
            ('/cmd_vel_recharge', '/cmd_vel_recharge'),
        ],
    )

    soft_avoidance_node = Node(
        package='mobile_robot',
        executable='soft_obstacle_avoidance_node',
        name='soft_obstacle_avoidance_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
        remappings=[
            # soft obstacle avoidance must publish its own candidate output
            ('/cmd_vel', '/cmd_vel_soft_avoid'),
        ],
    )

    return LaunchDescription([
        bridge_odom,

        # Supervisor first so arbitration is active before any candidate publishers start
        TimerAction(period=0.2, actions=[supervisor_node]),

        # Battery signals early (so supervisor has state)
        TimerAction(period=0.5, actions=[battery_node]),
        TimerAction(period=0.7, actions=[auto_recharge_node]),
        TimerAction(period=0.9, actions=[soft_avoidance_node]),

        # Then people behavior stack
        TimerAction(period=1.2, actions=[find_node]),
        TimerAction(period=1.4, actions=[move_node]),
        TimerAction(period=1.6, actions=[avoid_node]),
    ])