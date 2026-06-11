#!/usr/bin/env python3
# Copyright (c) 2024, Smart Farm Patrol Project - 꼬마 로봇 두리
# SPDX-License-Identifier: Apache-2.0
"""
nav_to_cow.py
=============
소(ground-truth) 접근 노드 — map→cow TF 를 읽어 소 앞 접근점으로
Nav2 NavigateToPose 목표를 전송. (시스템 ROS2, rclpy)

실행 (시스템 ROS2, 도메인 153):
  source /opt/ros/humble/setup.bash
  source ~/dev_ws/spot_ws/install/setup.bash
  export ROS_DOMAIN_ID=153
  python3 ~/dev_ws/spot_ws/src/smart_farm_spot/nav_to_cow.py
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
import tf2_ros

APPROACH = 1.2   # 소 앞 접근 거리(m)
GOAL_TOL = 0.4   # 목표 근처면 재전송 안 함


class NavToCow(Node):
    def __init__(self):
        super().__init__("nav_to_cow")
        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf, self)
        self.nav = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.sent = False
        self.create_timer(1.0, self.tick)
        self.get_logger().info("nav_to_cow 시작 — map→cow TF 대기 중")

    def _xy(self, parent, child):
        t = self.tf_buf.lookup_transform(parent, child, rclpy.time.Time())
        return t.transform.translation.x, t.transform.translation.y

    def tick(self):
        if self.sent:
            return
        try:
            cx, cy = self._xy("map", "cow")
        except Exception as e:
            self.get_logger().warn(f"cow TF 대기: {e}", throttle_duration_sec=3.0)
            return
        try:
            rx, ry = self._xy("map", "base_link")
        except Exception:
            rx, ry = 0.0, 0.0

        d = math.hypot(cx - rx, cy - ry)
        ux, uy = (1.0, 0.0) if d < 1e-3 else ((rx - cx) / d, (ry - cy) / d)
        ax, ay = cx + ux * APPROACH, cy + uy * APPROACH   # 소→로봇 방향으로 APPROACH
        yaw = math.atan2(cy - ay, cx - ax)                 # 소를 바라봄

        if not self.nav.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn("Nav2 navigate_to_pose 서버 대기")
            return

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(ax)
        goal.pose.pose.position.y = float(ay)
        goal.pose.pose.orientation.z = math.sin(yaw / 2)
        goal.pose.pose.orientation.w = math.cos(yaw / 2)

        fut = self.nav.send_goal_async(goal)
        fut.add_done_callback(self._on_resp)
        self.sent = True
        self.get_logger().info(
            f"🐄 소 접근 목표 전송: ({ax:.2f},{ay:.2f}) yaw={math.degrees(yaw):.0f}° "
            f"(소@{cx:.1f},{cy:.1f})")

    def _on_resp(self, fut):
        gh = fut.result()
        if not gh.accepted:
            self.get_logger().error("목표 거부됨"); self.sent = False; return
        self.get_logger().info("목표 수락됨 — 소로 이동 중")
        gh.get_result_async().add_done_callback(self._on_done)

    def _on_done(self, fut):
        self.get_logger().info("✅ 소 접근 완료")


def main():
    rclpy.init()
    node = NavToCow()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
