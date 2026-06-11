# Copyright (c) 2024, Smart Farm Patrol Project - 꼬마 로봇 두리
# SPDX-License-Identifier: Apache-2.0
"""
spot_nav2.launch.py
===================
Spot Nav2 전체 스택 실행 런치 (smart_farm_spot 통합 패키지).

구성:
  map_server, amcl, controller(DWB), planner(NavFn),
  behavior(복구), bt_navigator, waypoint_follower,
  velocity_smoother, lifecycle_manager

[실행]
  ros2 launch smart_farm_spot spot_nav2.launch.py
  ros2 launch smart_farm_spot spot_nav2.launch.py map:=/path/barn_map.yaml use_amcl:=false
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    pkg_share = get_package_share_directory("smart_farm_spot")
    default_params = os.path.join(pkg_share, "config", "nav2_params.yaml")
    default_map = os.path.join(pkg_share, "maps", "environment_final.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    map_yaml = LaunchConfiguration("map")
    use_amcl = LaunchConfiguration("use_amcl")
    autostart = LaunchConfiguration("autostart")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")

    declare_args = [
        DeclareLaunchArgument("use_sim_time", default_value="true",
                              description="Isaac Sim 시간 사용"),
        DeclareLaunchArgument("params_file", default_value=default_params,
                              description="Nav2 파라미터 파일"),
        DeclareLaunchArgument("map", default_value=default_map,
                              description="맵 yaml 파일"),
        DeclareLaunchArgument("use_amcl", default_value="true",
                              description="AMCL 위치 추정 사용 (false면 map=odom static 필요)"),
        DeclareLaunchArgument("autostart", default_value="true",
                              description="lifecycle 자동 시작"),
        DeclareLaunchArgument("cmd_vel_topic", default_value="cmd_vel",
                              description="velocity_smoother 최종 출력 토픽 "
                                          "(twist_mux 사용 시 nav2/cmd_vel 로 지정)"),
    ]

    # AMCL 사용 여부에 따른 lifecycle 노드 목록
    base_nodes = [
        "map_server", "controller_server", "planner_server",
        "behavior_server", "bt_navigator", "waypoint_follower",
        "velocity_smoother",
    ]

    nav2_group = GroupAction([
        SetParameter("use_sim_time", use_sim_time),

        # 맵 서버
        Node(
            package="nav2_map_server", executable="map_server",
            name="map_server", output="screen",
            parameters=[params_file, {"yaml_filename": map_yaml}],
        ),

        # AMCL (use_amcl=true 일 때만)
        Node(
            package="nav2_amcl", executable="amcl",
            name="amcl", output="screen",
            parameters=[params_file],
            condition=IfCondition(use_amcl),
        ),

        # 컨트롤러 (DWB) — cmd_vel은 smoother로
        Node(
            package="nav2_controller", executable="controller_server",
            name="controller_server", output="screen",
            parameters=[params_file],
            remappings=[("cmd_vel", "cmd_vel_nav")],
        ),

        # 전역 플래너
        Node(
            package="nav2_planner", executable="planner_server",
            name="planner_server", output="screen",
            parameters=[params_file],
        ),

        # 복구 행동
        Node(
            package="nav2_behaviors", executable="behavior_server",
            name="behavior_server", output="screen",
            parameters=[params_file],
        ),

        # BT 네비게이터
        Node(
            package="nav2_bt_navigator", executable="bt_navigator",
            name="bt_navigator", output="screen",
            parameters=[params_file],
        ),

        # 웨이포인트 팔로워
        Node(
            package="nav2_waypoint_follower", executable="waypoint_follower",
            name="waypoint_follower", output="screen",
            parameters=[params_file],
        ),

        # 속도 평활화 → 최종 /cmd_vel 발행
        Node(
            package="nav2_velocity_smoother", executable="velocity_smoother",
            name="velocity_smoother", output="screen",
            parameters=[params_file],
            remappings=[("cmd_vel", "cmd_vel_nav"),
                        ("cmd_vel_smoothed", cmd_vel_topic)],
        ),

        # 생명주기 관리 (AMCL 포함 버전)
        Node(
            package="nav2_lifecycle_manager", executable="lifecycle_manager",
            name="lifecycle_manager_navigation", output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "autostart": autostart,
                "node_names": base_nodes + ["amcl"],
            }],
            condition=IfCondition(use_amcl),
        ),
        # 생명주기 관리 (AMCL 없는 버전)
        Node(
            package="nav2_lifecycle_manager", executable="lifecycle_manager",
            name="lifecycle_manager_navigation", output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "autostart": autostart,
                "node_names": base_nodes,
            }],
            condition=UnlessCondition(use_amcl),
        ),
    ])

    return LaunchDescription(declare_args + [nav2_group])
