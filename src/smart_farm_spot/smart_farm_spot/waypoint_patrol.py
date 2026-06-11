#!/usr/bin/env python3
# Copyright (c) 2024, Smart Farm Patrol Project - 꼬마 로봇 두리
# SPDX-License-Identifier: Apache-2.0
"""
waypoint_patrol.py
==================
뒤쪽 통로(Rear Aisle) 자율 순찰 노드.

소들이 사료를 먹는 동안 뒤쪽 통로를 따라 웨이포인트를 순회하며,
각 웨이포인트에서 정지(촬영 시간 확보) → 다음 지점으로 이동.

Nav2 FollowWaypoints 액션을 사용하며, Nav2 내부 행동 트리가
소 돌발 접근 시 자동으로 충돌 회피(후진/회피)를 수행.

[실행]
  ros2 run spot_nav2 waypoint_patrol
  ros2 run spot_nav2 waypoint_patrol --ros-args -p loop:=true
  ros2 run spot_nav2 waypoint_patrol --ros-args -p waypoints_file:=/path/waypoints.yaml

[파라미터]
  loop            (bool)   무한 순찰 여부 (기본 false)
  waypoints_file  (string) 웨이포인트 yaml 경로 (미지정 시 내장 기본값)
  goal_frame      (string) 웨이포인트 좌표계 (기본 "map")
"""

import math

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

import yaml
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints


# ══════════════════════════════════════════════════════════════════════
# 내장 기본 웨이포인트 (map 좌표계, 단위 m) — (x, y, yaw_degrees)
#   waypoints_file 파라미터로 외부 yaml 지정 시 대체됨
# ══════════════════════════════════════════════════════════════════════
DEFAULT_WAYPOINTS = [
    (1.0,  0.0,   0.0),     # 통로 진입
    (3.0,  0.0,   0.0),     # 소 1 후면 관찰
    (5.0,  0.0,   0.0),     # 소 2 후면 관찰
    (7.0,  0.0,   0.0),     # 소 3 후면 관찰
    (8.0,  0.0,  90.0),     # 통로 끝 회전
    (8.0,  2.0, 180.0),     # 복귀 경로
    (1.0,  2.0, 180.0),     # 시작점 근처
    (0.0,  0.0, -90.0),     # 대기 위치 복귀
]


def make_pose(x, y, yaw_deg, frame="map"):
    """(x, y, yaw) → PoseStamped."""
    pose = PoseStamped()
    pose.header.frame_id = frame
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.position.z = 0.0
    yaw = math.radians(float(yaw_deg))
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


def load_waypoints_from_yaml(path):
    """
    yaml에서 웨이포인트 로드.

    형식:
      waypoints:
        - {x: 1.0, y: 0.0, yaw: 0.0}
        - {x: 3.0, y: 0.0, yaw: 0.0}
    """
    with open(path) as f:
        data = yaml.safe_load(f)
    wps = []
    for w in data.get("waypoints", []):
        wps.append((w["x"], w["y"], w.get("yaw", 0.0)))
    return wps


class WaypointPatrol(Node):
    """Nav2 FollowWaypoints로 뒤쪽 통로 순찰."""

    def __init__(self):
        super().__init__("waypoint_patrol")

        # ── 파라미터 선언 ────────────────────────────────────────
        self.declare_parameter("loop", False)
        self.declare_parameter("waypoints_file", "")
        self.declare_parameter("goal_frame", "map")

        self.loop = self.get_parameter("loop").value
        wp_file = self.get_parameter("waypoints_file").value
        self.frame = self.get_parameter("goal_frame").value

        # ── 웨이포인트 로드 ──────────────────────────────────────
        if wp_file:
            try:
                self.waypoints = load_waypoints_from_yaml(wp_file)
                self.get_logger().info(f"웨이포인트 {len(self.waypoints)}개 로드: {wp_file}")
            except Exception as e:
                self.get_logger().error(f"웨이포인트 파일 로드 실패: {e}. 기본값 사용.")
                self.waypoints = DEFAULT_WAYPOINTS
        else:
            self.waypoints = DEFAULT_WAYPOINTS
            self.get_logger().info(f"내장 기본 웨이포인트 {len(self.waypoints)}개 사용")

        self.patrol_count = 0
        self.client = ActionClient(self, FollowWaypoints, "follow_waypoints")

        self.get_logger().info(f"순찰 노드 시작 (loop={self.loop}). Nav2 서버 대기...")
        if not self.client.wait_for_server(timeout_sec=30.0):
            self.get_logger().error("Nav2 follow_waypoints 서버 미응답. 종료.")
            raise RuntimeError("Nav2 server timeout")
        self.get_logger().info("Nav2 연결됨. 순찰 시작.")
        self._start_patrol()

    def _start_patrol(self):
        goal = FollowWaypoints.Goal()
        goal.poses = [make_pose(x, y, yaw, self.frame) for x, y, yaw in self.waypoints]

        self.patrol_count += 1
        self.get_logger().info(
            f"[순찰 {self.patrol_count}회차] 웨이포인트 {len(goal.poses)}개 전송"
        )

        send_future = self.client.send_goal_async(goal, feedback_callback=self._feedback_cb)
        send_future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("순찰 목표 거부됨")
            rclpy.shutdown()
            return
        self.get_logger().info("순찰 목표 수락됨")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

    def _feedback_cb(self, feedback_msg):
        current = feedback_msg.feedback.current_waypoint
        total = len(self.waypoints)
        self.get_logger().info(
            f"  → 웨이포인트 {current + 1}/{total} 이동 중",
            throttle_duration_sec=2.0,
        )

    def _result_cb(self, future):
        result = future.result().result
        missed = result.missed_waypoints
        if missed:
            self.get_logger().warn(f"순찰 완료 (미도달 {len(missed)}개): {list(missed)}")
        else:
            self.get_logger().info("✅ 순찰 완료 (모든 웨이포인트 도달)")

        if self.loop:
            self.get_logger().info("순찰 반복...")
            self._start_patrol()
        else:
            self.get_logger().info("순찰 종료")
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = WaypointPatrol()
        rclpy.spin(node)
    except (KeyboardInterrupt, RuntimeError):
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
