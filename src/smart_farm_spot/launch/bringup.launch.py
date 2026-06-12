# Copyright (c) 2024, Smart Farm Patrol Project - 꼬마 로봇 두리
# SPDX-License-Identifier: Apache-2.0
"""
bringup.launch.py
=================
스마트 축사 자율순찰 통합 bringup (ROS 2 측 전체).

한 번의 launch로 ROS 2 쪽 기능을 인자로 켜고끕니다(전부 선택, 기본 nav2만):
  1. Nav2 전체 스택              (spot_nav2.launch.py 포함, 항상)
  2. 웨이포인트 순찰             patrol:=true   (waypoint_patrol)
  3. twist_mux 우선순위 중재     twist_mux:=true (Nav2→nav2/cmd_vel, mux→/cmd_vel)
  4. 웹 대시보드 브리지          dashboard:=true (dashboard_bridge)
  5. Keepout 투명벽 필터         keepout:=true   (keepout.launch.py)
  6. YOLO 검출 + 파행            yolo:=true      (/yolo/annotated, /cow/lameness)

  ※ 열화상 후처리(thermal_processor)는 비전 준비중이라 통합에서 제외됨.

⚠️  Isaac Sim 측 브릿지는 별도 실행해야 합니다 (SimulationApp — venv/isaaclab):
      ./isaaclab.sh -p .../isaac/scenario.py     (RL 보행 + 센서 + 카메라 + 소)

[실행]
  ros2 launch smart_farm_spot bringup.launch.py
  # 전 기능 통합 기동:
  ros2 launch smart_farm_spot bringup.launch.py \
      patrol:=true loop:=true twist_mux:=true dashboard:=true keepout:=true yolo:=true
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
    use_yolo = LaunchConfiguration("yolo")

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
        DeclareLaunchArgument("yolo", default_value="false",
                              description="YOLO 검출+파행(yolo_view) 노드 실행 "
                                          "(/yolo/annotated, /cow/lameness)"),
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

    # ── YOLO 검출 + 파행 (선택) — /yolo/annotated, /cow/lameness ─────
    yolo = Node(
        package="smart_farm_spot",
        executable="yolo_view",
        name="yolo_view",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(use_yolo),
    )

    return LaunchDescription(
        declare_args + [nav2, patrol, twist_mux, dashboard_bridge, keepout, yolo])
