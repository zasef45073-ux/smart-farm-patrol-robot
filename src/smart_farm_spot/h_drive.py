#!/usr/bin/env python3
# Copyright (c) 2024, Smart Farm Patrol Project - 꼬마 로봇 두리
# SPDX-License-Identifier: Apache-2.0
"""
h_drive.py
==========
H자 코스 순차 주행 — FollowWaypoints(waypoint_follower) 가 멈추는 환경에서,
검증된 NavigateToPose 액션을 웨이포인트마다 "순차" 전송해 한 점씩 주행.

각 웨이포인트:  NavigateToPose 전송 → 도달(SUCCEEDED) 대기 → 다음 점.

[실행]
  source ~/dev_ws/spot_ws/install/setup.bash   (또는 그냥 python3)
  ROS_DOMAIN_ID=153 python3 h_drive.py
  ROS_DOMAIN_ID=153 python3 h_drive.py --loop
  ROS_DOMAIN_ID=153 python3 h_drive.py --waypoints /path/to.yaml
"""
import argparse
import math
import os

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

import yaml
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

_DEFAULT_WP = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "config", "waypoints_h.yaml")


def make_pose(x, y, yaw_deg, frame="map"):
    p = PoseStamped()
    p.header.frame_id = frame
    p.pose.position.x = float(x)
    p.pose.position.y = float(y)
    yaw = math.radians(float(yaw_deg))
    p.pose.orientation.z = math.sin(yaw / 2.0)
    p.pose.orientation.w = math.cos(yaw / 2.0)
    return p


def load_waypoints(path):
    with open(path) as f:
        data = yaml.safe_load(f)
    return [(w["x"], w["y"], w.get("yaw", 0.0)) for w in data.get("waypoints", [])]


class HDrive(Node):
    def __init__(self):
        super().__init__("h_drive")
        self.declare_parameter("loop", False)
        self.declare_parameter("waypoints", _DEFAULT_WP)
        self.loop = self.get_parameter("loop").value
        wp_file = self.get_parameter("waypoints").value

        self.waypoints = load_waypoints(wp_file)
        self.get_logger().info(f"H 코스 {len(self.waypoints)}개 로드: {wp_file}")

        self.client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        if not self.client.wait_for_server(timeout_sec=30.0):
            self.get_logger().error("navigate_to_pose 서버 미응답. 종료.")
            raise RuntimeError("Nav2 server timeout")
        self.get_logger().info("Nav2 연결됨. H 순차 주행 시작.")

        self.idx = 0
        self.round = 1
        self._send_next()

    def _send_next(self):
        if self.idx >= len(self.waypoints):
            if self.loop:
                self.round += 1
                self.idx = 0
                self.get_logger().info(f"🔁 {self.round}회차 시작")
            else:
                self.get_logger().info("✅ H 코스 완료")
                rclpy.shutdown()
                return
        x, y, yaw = self.waypoints[self.idx]
        goal = NavigateToPose.Goal()
        goal.pose = make_pose(x, y, yaw)
        self.get_logger().info(
            f"[{self.idx + 1}/{len(self.waypoints)}] → ({x:.1f},{y:.1f},{yaw:.0f}°) 전송")
        fut = self.client.send_goal_async(goal)
        fut.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        gh = future.result()
        if not gh.accepted:
            self.get_logger().error("목표 거부됨 — 다음 점으로")
            self.idx += 1
            self._send_next()
            return
        gh.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, future):
        status = future.result().status
        # 4 = SUCCEEDED
        if status == 4:
            self.get_logger().info(f"  ✅ {self.idx + 1}번 도달")
        else:
            self.get_logger().warn(f"  ⚠️ {self.idx + 1}번 실패(status={status}) — 계속")
        self.idx += 1
        self._send_next()


def main(args=None):
    rclpy.init(args=args)
    try:
        rclpy.spin(HDrive())
    except (KeyboardInterrupt, RuntimeError):
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
