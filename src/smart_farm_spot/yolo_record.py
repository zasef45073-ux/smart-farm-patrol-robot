#!/usr/bin/env python3
# Copyright (c) 2024, Smart Farm Patrol Project - 꼬마 로봇 두리
# SPDX-License-Identifier: Apache-2.0
"""
yolo_record.py — /yolo/annotated 스트림을 mp4 로 녹화(시각 데모용).
  REC_OUT(기본 /tmp/vision_tail.mp4), REC_SECS(기본 30) 동안 녹화 후 종료.
실행: ROS_DOMAIN_ID=153 python3 yolo_record.py
"""
import os
import time
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

OUT = os.environ.get("REC_OUT", "/tmp/vision_tail.mp4")
SECS = float(os.environ.get("REC_SECS", "30"))
FPS = float(os.environ.get("REC_FPS", "15"))


class Rec(Node):
    def __init__(self):
        super().__init__("yolo_record")
        self.b = CvBridge()
        self.w = None
        self.n = 0
        self.t0 = time.time()
        self.create_subscription(Image, "/yolo/annotated", self.cb, qos_profile_sensor_data)
        self.get_logger().info(f"녹화 시작 → {OUT} ({SECS:.0f}s)")
        self.timer = self.create_timer(1.0, self._chk)

    def cb(self, m):
        try:
            f = self.b.imgmsg_to_cv2(m, "bgr8")
        except Exception:
            return
        if self.w is None:
            h, wd = f.shape[:2]
            self.w = cv2.VideoWriter(OUT, cv2.VideoWriter_fourcc(*"mp4v"), FPS, (wd, h))
        self.w.write(f)
        self.n += 1

    def _chk(self):
        if time.time() - self.t0 >= SECS:
            if self.w is not None:
                self.w.release()
            self.get_logger().info(f"녹화 완료: {OUT} ({self.n}프레임)")
            rclpy.shutdown()


def main():
    rclpy.init()
    try:
        rclpy.spin(Rec())
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
