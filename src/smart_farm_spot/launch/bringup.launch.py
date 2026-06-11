# Copyright (c) 2024, Smart Farm Patrol Project - 꼬마 로봇 두리
# SPDX-License-Identifier: Apache-2.0
"""
bringup.launch.py
=================
스마트 축사 자율순찰 통합 bringup (ROS 2 측 전체).

한 번의 launch로 ROS 2 쪽을 모두 띄웁니다:
  1. Nav2 전체 스택           (spot_nav2.launch.py 포함)
  2. 웨이포인트 순찰 노드       (waypoint_patrol, 선택)

  ※ 열화상 후처리(thermal_processor)는 비전 준비중이라 통합에서 제외됨.

⚠️  Isaac Sim 측 브릿지는 별도 실행해야 합니다 (SimulationApp 필요):
      ~/isaacsim/python.sh \
        $(ros2 pkg prefix smart_farm_spot)/share/smart_farm_spot/isaac/isaac_sim_bridge.py \
        --usd /path/to/barn_scene.usd --robot-prim /World/Spot --mode kinematic

[실행]
  ros2 launch smart_farm_spot bringup.launch.py
  ros2 launch smart_farm_spot bringup.launch.py use_amcl:=false patrol:=true loop:=true
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("smart_farm_spot")
    nav2_launch = os.path.join(pkg_share, "launch", "spot_nav2.launch.py")
    keepout_launch = os.path.join(pkg_share, "launch", "keepout.launch.py")
    default_waypoints = os.path.join(pkg_share, "config", "waypoints.yaml")
    twist_mux_cfg = os.path.join(pkg_share, "config", "twist_mux.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_amcl = LaunchConfiguration("use_amcl")
    map_yaml = LaunchConfiguration("map")
    enable_patrol = LaunchConfiguration("patrol")
    loop = LaunchConfiguration("loop")
    waypoints_file = LaunchConfiguration("waypoints_file")
    use_twist_mux = LaunchConfiguration("twist_mux")
    use_dashboard = LaunchConfiguration("dashboard")
    use_keepout = LaunchConfiguration("keepout")

    declare_args = [
        DeclareLaunchArgument("use_sim_time", default_value="true",
                              description="Isaac Sim 시간 사용"),
        DeclareLaunchArgument("use_amcl", default_value="true",
                              description="AMCL 위치 추정 사용"),
        DeclareLaunchArgument("map", default_value="",
                              description="맵 yaml (미지정 시 패키지 기본 맵)"),
        DeclareLaunchArgument("patrol", default_value="false",
                              description="웨이포인트 순찰 노드 자동 실행"),
        DeclareLaunchArgument("loop", default_value="false",
                              description="순찰 무한 반복"),
        DeclareLaunchArgument("waypoints_file", default_value=default_waypoints,
                              description="순찰 웨이포인트 yaml"),
        DeclareLaunchArgument("twist_mux", default_value="false",
                              description="twist_mux 우선순위 중재 사용 "
                                          "(Nav2 출력→nav2/cmd_vel, mux→/cmd_vel)"),
        DeclareLaunchArgument("dashboard", default_value="false",
                              description="웹 대시보드 브리지 노드 실행"),
        DeclareLaunchArgument("keepout", default_value="false",
                              description="Keepout 필터 서버 실행(투명벽 회피)"),
    ]

    # ── Nav2 스택 (spot_nav2.launch.py) ──────────────────────────────
    # map 인자가 비어있으면 하위 런치의 기본 맵을 쓰도록 전달하지 않음.
    # twist_mux 사용 시 Nav2 최종출력을 nav2/cmd_vel 로 돌려 mux 가 중재하게 함.
    nav2_cmd_vel = PythonExpression(
        ["'nav2/cmd_vel' if '", use_twist_mux, "' == 'true' else 'cmd_vel'"])
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "use_amcl": use_amcl,
            "map": map_yaml,
            "cmd_vel_topic": nav2_cmd_vel,
        }.items(),
    )

    # ── 웨이포인트 순찰 노드 (선택) ──────────────────────────────────
    patrol = Node(
        package="smart_farm_spot",
        executable="waypoint_patrol",
        name="waypoint_patrol",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "loop": loop,
            "waypoints_file": waypoints_file,
        }],
        condition=IfCondition(enable_patrol),
    )

    # ── twist_mux 우선순위 중재 (선택) — 출력 cmd_vel_out → /cmd_vel ──
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        name="twist_mux",
        output="screen",
        parameters=[twist_mux_cfg, {"use_sim_time": use_sim_time}],
        remappings=[("cmd_vel_out", "cmd_vel")],
        condition=IfCondition(use_twist_mux),
    )

    # ── 웹 대시보드 브리지 (선택) — /patrol/command ↔ /robot/status ──
    dashboard_bridge = Node(
        package="smart_farm_spot",
        executable="dashboard_bridge",
        name="dashboard_bridge",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(use_dashboard),
    )

    # ── Keepout 필터 서버 (선택, 투명벽 회피) ────────────────────────
    keepout = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(keepout_launch),
        condition=IfCondition(use_keepout),
    )

    return LaunchDescription(
        declare_args + [nav2, patrol, twist_mux, dashboard_bridge, keepout])
