#!/usr/bin/env python3
# Copyright (c) 2024, Smart Farm Patrol Project - 꼬마 로봇 두리
# SPDX-License-Identifier: Apache-2.0
"""
dashboard_bridge.py
===================
**웹 대시보드 ↔ 로봇 브리지** — FastAPI 대시보드(ros2_bridge.py)와 연결.

구독: /patrol/command (std_msgs/String, JSON {"command":"start|estop","course":..})
발행: /robot/status  (std_msgs/String, JSON {"status":"patrolling|idle|estop","course":..})
       /patrol/start  (std_msgs/Bool) — 순찰 노드가 소비(시작 트리거)
       e_stop          (std_msgs/Bool) — twist_mux 비상정지 lock

대시보드 흐름:
  [웹 순찰 버튼] → server/ros2_bridge → /patrol/command → (여기) → /patrol/start + status
  [웹 비상정지]  → /api/robot/estop → /patrol/command(estop) → (여기) → e_stop lock

실행: ROS_DOMAIN_ID=153 python3 dashboard_bridge.py
"""
import json

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String, Bool
    _ROS = True
except ImportError:                 # 순수 함수만 테스트할 때(rclpy 없음)
    _ROS = False
    Node = String = Bool = object


# ── 순수 로직(테스트 대상) ────────────────────────────────────────────
def parse_command(data: str) -> dict:
    """대시보드 명령 JSON 파싱 → {'command':..,'course':..}. 깨지면 command=None."""
    try:
        d = json.loads(data)
    except (ValueError, TypeError):
        return {"command": None, "course": None}
    return {"command": d.get("command"), "course": d.get("course")}


def build_status(status: str, course=None) -> str:
    """로봇 상태 → 대시보드용 JSON 문자열."""
    return json.dumps({"status": status, "course": course})


class DashboardBridge(Node):
    def __init__(self):
        super().__init__("dashboard_bridge")
        self.create_subscription(String, "/patrol/command", self._on_cmd, 10)
        self.status_pub = self.create_publisher(String, "/robot/status", 10)
        self.start_pub = self.create_publisher(Bool, "/patrol/start", 10)
        self.estop_pub = self.create_publisher(Bool, "e_stop", 10)
        self._status = "idle"
        self._course = None
        self.create_timer(2.0, self._pub_status)   # 주기 상태 보고
        self.get_logger().info("대시보드 브리지 시작 — /patrol/command 수신, /robot/status 발행")

    def _on_cmd(self, msg: String):
        c = parse_command(msg.data)
        cmd = c["command"]
        if cmd == "start":
            self._status, self._course = "patrolling", c["course"]
            self.start_pub.publish(Bool(data=True))
            self.get_logger().info(f"순찰 시작 명령 → /patrol/start (코스 {c['course']})")
        elif cmd == "estop":
            self._status, self._course = "estop", None
            self.estop_pub.publish(Bool(data=True))   # twist_mux lock
            self.get_logger().warn("비상정지 명령 → e_stop lock 발행")
        else:
            self.get_logger().warn(f"알 수 없는 명령: {msg.data}")
        self._pub_status()

    def _pub_status(self):
        self.status_pub.publish(String(data=build_status(self._status, self._course)))


def main():
    rclpy.init()
    try:
        rclpy.spin(DashboardBridge())
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
