#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    # Launch args (so you can override from command line)
    use_sim_time = LaunchConfiguration("use_sim_time")
    target_frame = LaunchConfiguration("target_frame")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation (/clock) time",
        ),
        DeclareLaunchArgument(
            "target_frame",
            default_value="odom",
            description="Frame to evaluate stationarity in (must exist in TF tree)",
        ),

        # --- Object Finder / Detector ---
        Node(
            package="mobile_robot",                  # <-- CHANGE if your package name differs
            executable="guest_find",               # <-- CHANGE if your executable name differs
            name="stationary_object_detector",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "target_frame": target_frame,

                # optional: keep your warmup param available
                "tf_warmup_grace_sec": 5.0,
            }],
        ),

        # --- Approacher ---
        Node(
            package="mobile_robot",                  # <-- CHANGE if your package name differs
            executable="guest_approach",           # <-- CHANGE if your executable name differs
            name="stationary_target_approach",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,

                # optional: match your topics if needed
                "stationary_topic": "/object/stationary",
                "range_topic": "/object/range",
                "bearing_topic": "/object/bearing",
                "cmd_vel_topic": "/cmd_vel",
            }],
        ),
    ])
