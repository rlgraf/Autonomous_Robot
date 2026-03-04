#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction



def generate_launch_description():

    pkg = get_package_share_directory('mobile_robot')

    # ── Read both YAMLs in Python — never pass battery YAML as --params-file ──
    battery_yaml_path   = os.path.join(pkg, 'parameters', 'battery_tunable_parameters.yaml')
    avoidance_yaml_path = os.path.join(pkg, 'parameters', 'avoidance_parameters.yaml')

    with open(battery_yaml_path, 'r') as f:
        battery_yaml = yaml.safe_load(f)

    bat  = battery_yaml['battery_node']['ros__parameters']
    rech = battery_yaml['auto_recharge_node']['ros__parameters']

    # ── Scalar params for battery_node (no nested lists/dicts) ───────────────
    battery_node_params = {
        'use_sim_time':        True,
        'battery_capacity_ah': bat['battery_capacity_ah'],
        'idle_drain_a':        bat['idle_drain_a'],
        'move_drain_per_ms':   bat['move_drain_per_ms'],
        'turn_drain_per_rads': bat['turn_drain_per_rads'],
        'battery_voltage':     bat['battery_voltage'],
        'publish_rate_hz':     bat['publish_rate_hz'],
        'charge_rate_a':       bat['charge_rate_a'],
        'charging_radius':     bat['charging_radius'],
        'stationary_thresh':   bat['stationary_thresh'],
        # color arrays and charging_stations are loaded inside battery_node.py
        # via yaml.safe_load() directly — not through ROS2 params
    }

    # ── Scalar params for auto_recharge_node ─────────────────────────────────
    recharge_node_params = {
        'use_sim_time':           True,
        'low_battery_threshold':  rech['low_battery_threshold'],
        'arrived_radius':         rech['arrived_radius'],
        'max_linear':             rech['max_linear'],
        'max_angular':            rech['max_angular'],
        'k_linear':               rech['k_linear'],
        'k_angular':              rech['k_angular'],
        'turn_only_angle':        rech['turn_only_angle'],
        'drive_angle':            rech['drive_angle'],
        'control_hz':             rech['control_hz'],
        # charging_stations loaded inside node via yaml.safe_load() — not ROS2 params
    }


    find_node = Node(
        package='mobile_robot',
        executable='identify6',
        name='identify6',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    move_node = Node(
        package='mobile_robot',
        executable='move6',
        name='move6',
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

    # ── avoidance_parameters.yaml is all scalars — safe to pass directly ─────
    battery_node = Node(
        package='mobile_robot',
        executable='battery_node',
        name='battery_node',
        output='screen',
        parameters=[battery_node_params],
    )

    auto_recharge_node = Node(
        package='mobile_robot',
        executable='auto_recharge_node',
        name='auto_recharge_node',
        output='screen',
        parameters=[recharge_node_params],
    )

    soft_avoidance_node = Node(
        package='mobile_robot',
        executable='soft_obstacle_avoidance_node',
        name='soft_obstacle_avoidance_node',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            avoidance_yaml_path,   # all scalars — safe for ROS2 parser
        ]
    )

    data_logger_node = Node(
        package='mobile_robot',
        executable='data_logger',
        name='data_logger',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    

    return LaunchDescription([
        bridge_odom,
        find_node,
        move_node,
        avoid_node,
        data_logger_node,
        TimerAction(period=1.0, actions=[battery_node]),
        TimerAction(period=1.0, actions=[auto_recharge_node]),
        TimerAction(period=1.0, actions=[soft_avoidance_node]),
    ])