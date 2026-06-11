#!/usr/bin/env python3
# Copyright (c) 2024, Smart Farm Patrol Project - 꼬마 로봇 두리
# SPDX-License-Identifier: Apache-2.0
"""
keepout.launch.py
=================
**투명(센서 미감지) 가상벽 회피** — Keepout 필터 서버 기동.

Nav2(nav2_params_slam) 의 KeepoutFilter 가 읽을:
  · filter_mask_server        : maps/keepout_mask.yaml → /keepout_filter_mask
  · costmap_filter_info_server: /costmap_filter_info

마스크는 먼저 생성:  python3 tools/make_keepout_mask.py
같이 실행:  ros2 launch smart_farm_spot keepout.launch.py
            (Nav2(nav2_params_slam) 와 동반 — 단독으론 효과 없음)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("smart_farm_spot")
    filter_params = os.path.join(pkg, "config", "keepout_filter.yaml")
    mask_yaml = os.path.join(pkg, "maps", "keepout_mask.yaml")

    return LaunchDescription([
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="filter_mask_server",
            output="screen",
            parameters=[filter_params, {"yaml_filename": mask_yaml}],
        ),
        Node(
            package="nav2_map_server",
            executable="costmap_filter_info_server",
            name="costmap_filter_info_server",
            output="screen",
            parameters=[filter_params],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_keepout",
            output="screen",
            parameters=[{
                "use_sim_time": True,
                "autostart": True,
                "node_names": ["filter_mask_server", "costmap_filter_info_server"],
            }],
        ),
    ])
