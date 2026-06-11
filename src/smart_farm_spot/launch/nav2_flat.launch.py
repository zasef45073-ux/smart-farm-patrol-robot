# Copyright (c) 2024, Smart Farm Patrol Project - 꼬마 로봇 두리
# SPDX-License-Identifier: Apache-2.0
"""
nav2_flat.launch.py
===================
평지 빈 코스용 Nav2 스택 (맵/AMCL 없음).

map→odom, odom→base_link TF 와 /odom 은 nav_bridge(Isaac) 가 발행하므로
map_server / amcl 없이 controller/planner/behavior/bt/waypoint/smoother 만 띄움.

[실행]
  ros2 launch smart_farm_spot nav2_flat.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    pkg_share = get_package_share_directory("smart_farm_spot")
    default_params = os.path.join(pkg_share, "config", "nav2_params_flat.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    autostart = LaunchConfiguration("autostart")

    declare_args = [
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("params_file", default_value=default_params),
        DeclareLaunchArgument("autostart", default_value="true"),
    ]

    lifecycle_nodes = [
        "controller_server", "planner_server", "behavior_server",
        "bt_navigator", "waypoint_follower", "velocity_smoother",
    ]

    group = GroupAction([
        SetParameter("use_sim_time", use_sim_time),

        Node(package="nav2_controller", executable="controller_server",
             name="controller_server", output="screen",
             parameters=[params_file],
             remappings=[("cmd_vel", "cmd_vel_nav")]),
        Node(package="nav2_planner", executable="planner_server",
             name="planner_server", output="screen", parameters=[params_file]),
        Node(package="nav2_behaviors", executable="behavior_server",
             name="behavior_server", output="screen", parameters=[params_file]),
        Node(package="nav2_bt_navigator", executable="bt_navigator",
             name="bt_navigator", output="screen", parameters=[params_file]),
        Node(package="nav2_waypoint_follower", executable="waypoint_follower",
             name="waypoint_follower", output="screen", parameters=[params_file]),
        Node(package="nav2_velocity_smoother", executable="velocity_smoother",
             name="velocity_smoother", output="screen", parameters=[params_file],
             remappings=[("cmd_vel", "cmd_vel_nav"),
                         ("cmd_vel_smoothed", "cmd_vel")]),

        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
             name="lifecycle_manager_navigation", output="screen",
             parameters=[{"use_sim_time": use_sim_time,
                          "autostart": autostart,
                          "node_names": lifecycle_nodes}]),
    ])

    return LaunchDescription(declare_args + [group])
