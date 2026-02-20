from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    common_params = [{"use_sim_time": True},]
    
    detector = Node(
        package='mobile_robot',
        executable='simple_guest_find',
        name='stationary_object_detector',
        output='screen',
        parameters=[
            {"scan_topic": "/scan"},
            {"world_frame": "odom"},
        ]
    )

    approach = Node(
        package='mobile_robot',
        executable='simple_guest_approach',
        name='stationary_target_approach',
        output='screen',
        parameters=[
            {"base_frame": "base_footprint"},
            {"cmd_vel_topic": "/cmd_vel"},
        ]
    )

    return LaunchDescription([
        detector,
        approach
    ])
