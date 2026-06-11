# Copyright (c) 2024, Smart Farm Patrol Project - 꼬마 로봇 두리
# SPDX-License-Identifier: Apache-2.0
"""
spot_slam.launch.py
===================
SLAM Toolbox로 축사 맵을 생성하는 런치 (사전 맵이 없을 때).

[실행]
  ros2 launch spot_nav2 spot_slam.launch.py

  # 매핑 후 맵 저장
  ros2 run nav2_map_server map_saver_cli -f ~/barn_map

[참고]
  매핑 중에는 teleop로 로봇 수동 주행 권장:
  ros2 run teleop_twist_keyboard teleop_twist_keyboard
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),

        Node(
            package="slam_toolbox",
            executable="async_slam_toolbox_node",
            name="slam_toolbox",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "odom_frame": "odom",
                "map_frame": "map",
                "base_frame": "base_link",
                "scan_topic": "/scan",
                "mode": "mapping",
                # 4족 보행 진동 대응: 업데이트 보수적
                "minimum_travel_distance": 0.3,
                "minimum_travel_heading": 0.3,
                "map_update_interval": 2.0,
                "resolution": 0.05,
                "max_laser_range": 30.0,
                "transform_publish_period": 0.05,
                "transform_timeout": 0.5,
                "do_loop_closing": True,
                "loop_search_maximum_distance": 5.0,
            }],
        ),
    ])
