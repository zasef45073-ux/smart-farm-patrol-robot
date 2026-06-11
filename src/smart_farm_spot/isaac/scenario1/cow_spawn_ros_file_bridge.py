#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
cow_spawn_ros_file_bridge.py

ROS2 토픽 <-> /tmp 명령 파일 브릿지.

왜 필요함?
  Isaac Sim 내부 Python이 3.11이고 ROS Humble rclpy는 Python 3.10용이라
  Isaac Sim 안에서 rclpy import가 실패함.
  그래서 ROS2 토픽 subscriber는 바깥 Python3.10에서 돌리고,
  Isaac Sim 쪽 스폰 스크립트는 /tmp/*.json 명령 파일을 감시함.

이 파일은 일반 ROS2 터미널에서 실행:
  source /opt/ros/humble/setup.bash
  export ROS_DOMAIN_ID=153
  python3 cow_spawn_ros_file_bridge.py
"""

import json
import os
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


SCENARIO1_CMD_TOPIC = "/scenario1/eat_cow_spawn_cmd"
SCENARIO1_STATUS_TOPIC = "/scenario1/eat_cow_spawn_status"
SCENARIO1_CMD_FILE = Path("/tmp/scenario1_eat_cow_cmd.json")
SCENARIO1_STATUS_FILE = Path("/tmp/scenario1_eat_cow_status.json")

SCENARIO2_CMD_TOPIC = "/scenario2/move_cow_spawn_cmd"
SCENARIO2_STATUS_TOPIC = "/scenario2/move_cow_spawn_status"
SCENARIO2_CMD_FILE = Path("/tmp/scenario2_move_cow_cmd.json")
SCENARIO2_STATUS_FILE = Path("/tmp/scenario2_move_cow_status.json")


class CowSpawnRosFileBridge(Node):
    def __init__(self):
        super().__init__("cow_spawn_ros_file_bridge")

        self.seq = 0
        self.last_status_mtime = {
            str(SCENARIO1_STATUS_FILE): 0.0,
            str(SCENARIO2_STATUS_FILE): 0.0,
        }

        self.scenario1_pub = self.create_publisher(String, SCENARIO1_STATUS_TOPIC, 10)
        self.scenario2_pub = self.create_publisher(String, SCENARIO2_STATUS_TOPIC, 10)

        self.scenario1_sub = self.create_subscription(
            String,
            SCENARIO1_CMD_TOPIC,
            lambda msg: self.write_cmd("scenario1", msg.data, SCENARIO1_CMD_FILE),
            10,
        )

        self.scenario2_sub = self.create_subscription(
            String,
            SCENARIO2_CMD_TOPIC,
            lambda msg: self.write_cmd("scenario2", msg.data, SCENARIO2_CMD_FILE),
            10,
        )

        self.timer = self.create_timer(0.2, self.publish_status_files)

        self.get_logger().info("CowSpawnRosFileBridge READY")
        self.get_logger().info(f"subscribe: {SCENARIO1_CMD_TOPIC}")
        self.get_logger().info(f"subscribe: {SCENARIO2_CMD_TOPIC}")
        self.get_logger().info(f"publish  : {SCENARIO1_STATUS_TOPIC}")
        self.get_logger().info(f"publish  : {SCENARIO2_STATUS_TOPIC}")

    def write_cmd(self, scenario_name: str, raw_cmd: str, cmd_file: Path):
        self.seq += 1

        payload = {
            "seq": self.seq,
            "scenario": scenario_name,
            "cmd": raw_cmd.strip() if raw_cmd else "spawn",
            "time": time.time(),
        }

        tmp_file = cmd_file.with_suffix(".tmp")
        tmp_file.write_text(json.dumps(payload, ensure_ascii=False), encoding="utf-8")
        os.replace(tmp_file, cmd_file)

        self.get_logger().info(f"{scenario_name} command written: {payload}")

    def read_status_file(self, status_file: Path):
        try:
            if not status_file.exists():
                return None

            mtime = status_file.stat().st_mtime
            key = str(status_file)

            if mtime <= self.last_status_mtime.get(key, 0.0):
                return None

            self.last_status_mtime[key] = mtime

            data = json.loads(status_file.read_text(encoding="utf-8"))
            return str(data.get("status", data))

        except Exception as e:
            return f"status read ERROR: {type(e).__name__}: {e}"

    def publish_status_files(self):
        s1 = self.read_status_file(SCENARIO1_STATUS_FILE)
        if s1 is not None:
            msg = String()
            msg.data = s1
            self.scenario1_pub.publish(msg)

        s2 = self.read_status_file(SCENARIO2_STATUS_FILE)
        if s2 is not None:
            msg = String()
            msg.data = s2
            self.scenario2_pub.publish(msg)


def main():
    rclpy.init()
    node = CowSpawnRosFileBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
