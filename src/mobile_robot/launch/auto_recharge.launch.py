###############################################################################
# Auto Recharge Launch File
#
# Reads battery_tunable_parameters.yaml with Python (not as --params-file)
# to avoid ROS2's C parser choking on nested lists (charging_stations) and
# nested dicts (color_*).  Only flat scalar values are passed to each node.
###############################################################################

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


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

    return LaunchDescription([
        TimerAction(period=1.0, actions=[battery_node]),
        TimerAction(period=1.0, actions=[auto_recharge_node]),
        TimerAction(period=1.0, actions=[soft_avoidance_node]),
    ])