#!/usr/bin/env python3
# Copyright (c) 2024, Smart Farm Patrol Project - 꼬마 로봇 두리
# SPDX-License-Identifier: Apache-2.0
"""
scenario_nav.py
===============
**순찰 + 소 후방 추적 통합 relay** — 시나리오 전체를 Nav2(+SLAM+RL)로 구동.

흐름:
  1. PATROL: scenario.py가 /tmp/patrol_waypoints.json 에 저장한 통로 끝들을
     NavigateToPose 로 순차 순회(우리 안까지 들어감).
  2. 순찰 중 scenario.py의 **in-Isaac YOLO 검출**이 소를 찾으면 /tmp/scenario_goal.json
     (cow_rear/cow, map 원점=로봇 시작점 상대)이 갱신됨.
  3. SEEK: 그 파일을 감지하면 순찰을 **선점(preempt)** 하고 소 후방(꼬리 1.5m 뒤)으로
     NavigateToPose, 소를 바라봄. 접근하며 검출이 갱신되면 목표 재전송(수렴).

경로계획·장애물회피=Nav2(라이다 코스트맵), 맵=SLAM, 보행=RL 정책(scenario.py).
검출·꼬리3D는 Isaac 내부, 밖으론 목표만 흐른다.

실행: ROS_DOMAIN_ID=153 python3 scenario_nav.py
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
GOAL_FILE = "/tmp/scenario_goal.json"
LOOPS = int(os.environ.get("SF_PATROL_LOOPS", "0"))            # 0=무한 순찰(검출까지)
RESEND_DIST = float(os.environ.get("SF_RESEND_DIST", "0.5"))  # 소후방 목표 변화 임계(m)


def make_pose(x, y, yaw, frame="map"):
    p = PoseStamped()
    p.header.frame_id = frame
    p.pose.position.x = float(x)
    p.pose.position.y = float(y)
    p.pose.orientation.z = math.sin(yaw / 2.0)
    p.pose.orientation.w = math.cos(yaw / 2.0)
    return p


class PatrolSeek(Node):
    def __init__(self):
        super().__init__("scenario_nav")
        self.mode = "PATROL"
        self.i = 0
        self.loop = 0
        self.seek_goal = None       # 마지막 전송한 소후방 (x,y)
        self.goal_mtime = 0.0       # scenario_goal.json 갱신 감지

        for _ in range(60):
            if os.path.exists(WP_FILE):
                break
            time.sleep(1.0)
        try:
            with open(WP_FILE) as f:
                self.wps = json.load(f)["waypoints"]
        except Exception:
            self.wps = []
        # 시작점(map 원점) 근처 웨이포인트 제거 — 경로길이0이면 DWB가 제자리 정체(메모리 함정).
        _skip = float(os.environ.get("SF_WP_SKIP_R", "1.5"))
        self.wps = [w for w in self.wps if math.hypot(w[0], w[1]) > _skip]

        self.client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        if not self.client.wait_for_server(timeout_sec=40.0):
            self.get_logger().error("navigate_to_pose 서버 미응답. 종료.")
            raise RuntimeError("Nav2 timeout")
        self.get_logger().info(
            f"순찰 {len(self.wps)}개 통로 끝 — 주행 시작. 소 검출 시 후방 추적 전환.")
        self.create_timer(0.5, self._watch_cow)   # 소 검출 감시
        self._send_patrol()

    # ── 소 검출 감시 → SEEK 전환/갱신 ──
    def _watch_cow(self):
        try:
            mt = os.path.getmtime(GOAL_FILE)
            with open(GOAL_FILE) as f:
                g = json.load(f)
        except Exception:
            return
        if "cow_rear" not in g or mt == self.goal_mtime:
            return
        self.goal_mtime = mt
        rear = g["cow_rear"]
        cow = g.get("cow", rear)
        yaw = math.atan2(cow[1] - rear[1], cow[0] - rear[0])   # 후방에서 소 바라봄
        if self.mode == "PATROL":
            self.mode = "SEEK"
            self.get_logger().info("★ 소 검출 — 순찰 선점, 후방 추적으로 전환.")
        if self.seek_goal and math.hypot(
                rear[0] - self.seek_goal[0], rear[1] - self.seek_goal[1]) < RESEND_DIST:
            return                                              # 거의 같은 목표 → 재전송 안 함
        self.seek_goal = (rear[0], rear[1])
        goal = NavigateToPose.Goal()
        goal.pose = make_pose(rear[0], rear[1], yaw)
        self.get_logger().info(
            f"▶ 소 후방({rear[0]:.2f},{rear[1]:.2f}) 소바라봄 {math.degrees(yaw):.0f}° 주행")
        self.client.send_goal_async(goal).add_done_callback(self._on_seek_resp)

    def _on_seek_resp(self, fut):
        gh = fut.result()
        if gh.accepted:
            gh.get_result_async().add_done_callback(self._on_seek_result)

    def _on_seek_result(self, fut):
        if fut.result().status == 4:
            self.get_logger().info("★ 소 후방(꼬리쪽) 도착! — 앉아서 검사 트리거")
            # scenario.py(Isaac)의 INSPECT(앉기→검사)를 발동: 트리거 파일 생성
            try:
                open(os.environ.get("SF_INSPECT_TRIGGER", "/tmp/scenario_inspect"), "w").close()
            except OSError as e:
                self.get_logger().warn(f"INSPECT 트리거 생성 실패: {e}")
        # SEEK 유지: 검출 갱신되면 _watch_cow가 재전송

    # ── 순찰 ──
    def _send_patrol(self):
        if self.mode != "PATROL" or not self.wps:
            return
        if self.i >= len(self.wps):
            self.loop += 1
            if LOOPS and self.loop >= LOOPS:
                self.get_logger().info(f"순찰 {self.loop}바퀴 완료(소 미검출).")
                return
            self.i = 0
            self.get_logger().info(f"── 순찰 {self.loop + 1}바퀴 ──")
        x, y = self.wps[self.i]
        nx, ny = self.wps[(self.i + 1) % len(self.wps)]
        yaw = math.atan2(ny - y, nx - x)
        self.get_logger().info(f"▶ 순찰[{self.i + 1}/{len(self.wps)}] ({x:.2f},{y:.2f}) 주행")
        goal = NavigateToPose.Goal()
        goal.pose = make_pose(x, y, yaw)
        self.client.send_goal_async(goal).add_done_callback(self._on_patrol_resp)

    def _on_patrol_resp(self, fut):
        gh = fut.result()
        if not gh.accepted:
            self.i += 1
            self._send_patrol()
            return
        gh.get_result_async().add_done_callback(self._on_patrol_result)

    def _on_patrol_result(self, fut):
        if self.mode != "PATROL":         # SEEK로 전환됐으면 순찰 중단
            return
        x, y = self.wps[self.i]
        if fut.result().status == 4:
            self.get_logger().info(f"  ✓ 통로 끝 ({x:.2f},{y:.2f}) 도착")
        self.i += 1
        self._send_patrol()


def main():
    rclpy.init()
    try:
        rclpy.spin(PatrolSeek())
    except (KeyboardInterrupt, RuntimeError):
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
