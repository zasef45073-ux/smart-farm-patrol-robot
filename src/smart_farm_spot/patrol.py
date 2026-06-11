#!/usr/bin/env python3
# Copyright (c) 2024, Smart Farm Patrol Project - 꼬마 로봇 두리
# SPDX-License-Identifier: Apache-2.0
"""
patrol.py
=========
순찰 — SLAM + Nav2 + RL 로 **축사 통로 끝을 전부** 순서대로 순회한다.

scenario.py(Isaac 브릿지)가 /tmp/patrol_waypoints.json 에 저장한
  waypoints : [[x,y], ...]  (map/odom 원점=로봇 시작점 기준 상대)
를 읽어, 각 웨이포인트로 NavigateToPose 를 순차 전송한다(도착/실패하면 다음으로).
마지막까지 돌면 처음으로 돌아가 반복 순찰(SF_PATROL_LOOPS 로 횟수 제한, 0=무한).
각 지점에서 yaw 는 다음 웨이포인트를 바라보게(전진 방향) 설정.

경로계획·장애물회피=Nav2(라이다 코스트맵), 맵=SLAM, 보행=RL 정책(scenario.py).

실행:
  ROS_DOMAIN_ID=153 python3 patrol.py
"""
import json
import math
import os
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

WP_FILE = "/tmp/patrol_waypoints.json"
LOOPS = int(os.environ.get("SF_PATROL_LOOPS", "0"))   # 0 = 무한 반복


def make_pose(x, y, yaw, frame="map"):
    p = PoseStamped()
    p.header.frame_id = frame
    p.pose.position.x = float(x)
    p.pose.position.y = float(y)
    p.pose.orientation.z = math.sin(yaw / 2.0)
    p.pose.orientation.w = math.cos(yaw / 2.0)
    return p


class Patrol(Node):
    def __init__(self):
        super().__init__("patrol")
        for _ in range(60):
            if os.path.exists(WP_FILE):
                break
            time.sleep(1.0)
        with open(WP_FILE) as f:
            self.wps = json.load(f)["waypoints"]
        if not self.wps:
            self.get_logger().error("웨이포인트 없음. 종료."); raise RuntimeError("no waypoints")
        self.get_logger().info(
            f"순찰 통로 끝 {len(self.wps)}개: "
            f"{[(round(x, 1), round(y, 1)) for x, y in self.wps]}")

        self.client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        if not self.client.wait_for_server(timeout_sec=40.0):
            self.get_logger().error("navigate_to_pose 서버 미응답. 종료."); raise RuntimeError("Nav2 timeout")
        self.get_logger().info("Nav2 연결됨 → 순찰 시작.")
        self.i = 0
        self.loop = 0
        self._send_next()

    def _send_next(self):
        if self.i >= len(self.wps):
            self.loop += 1
            if LOOPS and self.loop >= LOOPS:
                self.get_logger().info(f"★ 순찰 {self.loop}바퀴 완료 — 종료.")
                rclpy.shutdown(); return
            self.i = 0
            self.get_logger().info(f"── 순찰 {self.loop + 1}바퀴 시작 ──")
        x, y = self.wps[self.i]
        # 다음 지점을 바라보게(전진 방향) yaw
        nx, ny = self.wps[(self.i + 1) % len(self.wps)]
        yaw = math.atan2(ny - y, nx - x)
        self.get_logger().info(
            f"▶ [{self.i + 1}/{len(self.wps)}] 통로 끝 ({x:.2f},{y:.2f}) 로 순찰 주행")
        goal = NavigateToPose.Goal()
        goal.pose = make_pose(x, y, yaw)
        self.client.send_goal_async(goal).add_done_callback(self._on_resp)

    def _on_resp(self, future):
        gh = future.result()
        if not gh.accepted:
            self.get_logger().warn("목표 거부 → 다음 지점"); self.i += 1; self._send_next(); return
        gh.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, future):
        status = future.result().status
        x, y = self.wps[self.i]
        if status == 4:
            self.get_logger().info(f"  ✓ 통로 끝 ({x:.2f},{y:.2f}) 도착")
        else:
            self.get_logger().warn(f"  ! ({x:.2f},{y:.2f}) 주행 종료(status={status}) → 다음")
        self.i += 1
        self._send_next()


def main():
    rclpy.init()
    try:
        rclpy.spin(Patrol())
    except (KeyboardInterrupt, RuntimeError):
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
