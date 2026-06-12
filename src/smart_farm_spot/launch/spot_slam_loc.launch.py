# Copyright (c) 2024, Smart Farm Patrol Project - 꼬마 로봇 두리
# SPDX-License-Identifier: Apache-2.0
"""
spot_slam_loc.launch.py
=======================
**AMCL + SLAM 복합 = slam_toolbox 위치추정 모드** (map_and_localization 노드).

사전맵(.posegraph)을 로드해 **scan-match 로 위치추정(AMCL 역할)** 하면서
동시에 **새 관측을 맵에 반영(SLAM 역할)** — 한 노드가 둘을 합침.

전제: 직렬화 사전맵이 있어야 함(map_file_name, 확장자 없이).
  생성:  ① ros2 launch smart_farm_spot spot_slam.launch.py  (매핑 모드로 주행)
         ② ros2 service call /slam_toolbox/serialize_map \
              slam_toolbox/srv/SerializePoseGraph "{filename: <맵경로/barn_slam>}"
  그러면 barn_slam.posegraph + barn_slam.data 생성.

[실행]
  ros2 launch smart_farm_spot spot_slam_loc.launch.py
  ros2 launch smart_farm_spot spot_slam_loc.launch.py map:=/경로/barn_slam
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("smart_farm_spot")
    # 직렬화 맵 기본 경로(확장자 없이). serialize 로 barn_slam.posegraph/.data 생성해 둘 것.
    default_map = os.path.join(pkg, "maps", "barn_slam")

    use_sim_time = LaunchConfiguration("use_sim_time")
    map_file = LaunchConfiguration("map")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("map", default_value=default_map,
                              description="직렬화 사전맵 경로(확장자 없이)"),

        Node(
            package="slam_toolbox",
            # 매핑+위치추정 동시 노드 = AMCL(사전맵 추정)+SLAM(맵 갱신) 복합
            executable="map_and_localization_slam_toolbox_node",
            name="slam_toolbox",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "odom_frame": "odom",
                "map_frame": "map",
                "base_frame": "base_link",
                "scan_topic": "/scan",
                "mode": "localization",
                # 사전맵 로드 + 시작 자세(도크/원점). 맵 없으면 빈 맵에서 시작.
                "map_file_name": map_file,
                "map_start_at_dock": True,
                # 4족 보행 진동 대응(매핑과 동일 보수적 업데이트)
                "minimum_travel_distance": 0.3,
                "minimum_travel_heading": 0.3,
                "resolution": 0.05,
                "max_laser_range": 30.0,
                "transform_publish_period": 0.05,
                "transform_timeout": 0.5,
                "do_loop_closing": True,
                "loop_search_maximum_distance": 5.0,
            }],
        ),
    ])
