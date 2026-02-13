from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    avoider_node = Node(
        package='mobile_robot',
        executable='simple_avoid_obstacle',
        name='simple_avoid_obstacle',
        output='screen',

        # Optional parameters (override defaults in script)
        #parameters=[
        #    {'forward_speed': 0.25},
        #    {'turn_speed': 0.8},
        #    {'stop_distance': 0.7},
        #    {'front_angle_deg': 30.0},
        #],

        # Optional topic remapping if needed
        # remappings=[
        #     ('/scan', '/lidar/scan'),
        #     ('/cmd_vel', '/diff_drive/cmd_vel'),
        # ]
    )

    return LaunchDescription([
        avoider_node
    ])
