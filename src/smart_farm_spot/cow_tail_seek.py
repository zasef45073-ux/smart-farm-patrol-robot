#!/usr/bin/env python3
# Copyright (c) 2024, Smart Farm Patrol Project - 꼬마 로봇 두리
# SPDX-License-Identifier: Apache-2.0
"""
cow_tail_seek.py
================
**비전 기반 소 꼬리 후방 이동** — 정답좌표 X, 카메라 측정값으로 꼬리를 잡아 그 뒤로 간다.

파이프라인:
  1. /spot_cam/rgb 에서 YOLO-Pose 로 소 검출 + 14 키포인트
  2. kpt[5] = tail_base(꼬리) 픽셀 추출 (가시성 충분할 때만)
  3. /spot_cam/depth 에서 그 픽셀 깊이 d 샘플
  4. camera_info 내부파라미터로 (u,v,d) → 카메라 optical 3D 역투영
  5. TF (spot_cam → map) 로 꼬리 월드좌표 산출
  6. 꼬리에서 로봇쪽으로 BEHIND(1.5m) 물러난 점 = 후방 검사 목표, yaw=꼬리 바라봄
  7. Nav2 NavigateToPose 전송 (안정 추정 N회 평균 후, 크게 바뀌면 갱신)

전제: scenario.py 가 base_link→spot_cam(optical) TF 를 발행, map→odom(SLAM/known map) 존재.

실행:
  ROS_DOMAIN_ID=153 python3 cow_tail_seek.py
"""
import math
import os
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, PointStamped, Twist
from nav2_msgs.action import NavigateToPose
from cv_bridge import CvBridge
from ultralytics import YOLO
import tf2_ros
from tf2_geometry_msgs import do_transform_point

_PKG = os.path.dirname(os.path.abspath(__file__))
WEIGHTS = os.path.join(_PKG, "assets", "yolo", "best.pt")
TAIL_KPT = 5            # 0 nose..5 tail_base..
CONF = 0.40
KPT_CONF = 0.30
BEHIND = float(os.environ.get("SF_BEHIND_TAIL", "1.5"))
N_STABLE = 5           # 안정 추정에 필요한 측정 수
RESEND_DIST = 0.6      # 꼬리 추정이 이만큼 바뀌면 목표 갱신
MAP_FRAME = "map"


class CowTailSeek(Node):
    def __init__(self):
        super().__init__("cow_tail_seek")
        self.model = YOLO(WEIGHTS)
        self.bridge = CvBridge()
        self._depth = None
        self._K = None
        self._tails = []          # 최근 꼬리 월드 추정 누적
        self._goal_xy = None      # 마지막 전송 목표

        q = qos_profile_sensor_data
        self.create_subscription(Image, "/spot_cam/rgb", self.cb_rgb, q)
        self.create_subscription(Image, "/spot_cam/depth", self.cb_depth, q)
        self.create_subscription(CameraInfo, "/spot_cam/camera_info", self.cb_info, q)

        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf, self)
        self.client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self._nav_ready = False
        # 부트스트랩: 소가 안 보이면 제자리 회전하며 탐색(소 시야 진입 → 검출)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self._last_detect = 0.0
        self._goal_sent = False
        self._search_wz = float(os.environ.get("SF_SEARCH_WZ", "0.5"))
        self.create_timer(0.2, self._search)
        self.get_logger().info("비전 꼬리 추적 시작 — 소 탐색(회전)→tail_base(kpt5) 검출→뎁스→TF→후방 목표")

    def _search(self):
        # 꼬리 목표 진행중이면 Nav2 가 제어 → 탐색 중지
        if self._goal_sent:
            return
        # 방금 검출됐으면 회전 멈추고 안정 추정 대기
        if time.time() - self._last_detect < 1.0:
            self.cmd_pub.publish(Twist())
            return
        t = Twist()
        t.angular.z = self._search_wz   # 제자리 회전하며 소 탐색
        self.cmd_pub.publish(t)

    def cb_info(self, m):
        self._K = (m.k[0], m.k[4], m.k[2], m.k[5])   # fx, fy, cx, cy

    def cb_depth(self, m):
        try:
            self._depth = self.bridge.imgmsg_to_cv2(m, "32FC1")
        except Exception:
            pass

    def cb_rgb(self, m):
        if self._depth is None or self._K is None:
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(m, "bgr8")
        except Exception:
            return
        r = self.model(frame, conf=CONF, verbose=False)[0]
        if r.keypoints is None or r.boxes is None or len(r.boxes) == 0:
            return
        kps = r.keypoints.data.cpu().numpy()   # (n,14,3)
        # 가장 신뢰 높은 소
        bi = int(np.argmax(r.boxes.conf.cpu().numpy()))
        tail = kps[bi, TAIL_KPT]               # [x, y, conf]
        if float(tail[2]) < KPT_CONF:
            return
        u, v = float(tail[0]), float(tail[1])
        d = self._sample_depth(u, v, frame.shape)
        if d is None:
            return
        fx, fy, cx, cy = self._K
        # 카메라 optical 3D (z 전방, x 우, y 하)
        pt = PointStamped()
        pt.header.frame_id = m.header.frame_id or "spot_cam"
        pt.point.x = (u - cx) * d / fx
        pt.point.y = (v - cy) * d / fy
        pt.point.z = d
        # spot_cam → map
        try:
            tw = self.tf_buf.transform(pt, MAP_FRAME, timeout=rclpy.duration.Duration(seconds=0.2))
        except Exception as e:
            self.get_logger().warn(f"TF spot_cam→map 실패: {e}", throttle_duration_sec=3.0)
            return
        self._last_detect = time.time()   # 소 검출됨 → 탐색 회전 멈춤
        self._tails.append((tw.point.x, tw.point.y))
        if len(self._tails) > N_STABLE:
            self._tails.pop(0)
        if len(self._tails) >= N_STABLE:
            self._maybe_send()

    def _sample_depth(self, u, v, sh):
        dh, dw = self._depth.shape[:2]
        ah, aw = sh[:2]
        du, dv = int(u * dw / aw), int(v * dh / ah)
        if 0 <= dv < dh and 0 <= du < dw:
            dd = float(self._depth[dv, du])
            if np.isfinite(dd) and 0.1 < dd < 30.0:
                return dd
        return None

    def _robot_xy(self):
        try:
            t = self.tf_buf.lookup_transform(MAP_FRAME, "base_link", rclpy.time.Time())
            return t.transform.translation.x, t.transform.translation.y
        except Exception:
            return None

    def _maybe_send(self):
        tx = float(np.median([p[0] for p in self._tails]))
        ty = float(np.median([p[1] for p in self._tails]))
        rob = self._robot_xy()
        if rob is None:
            return
        rx, ry = rob
        dx, dy = tx - rx, ty - ry
        dist = math.hypot(dx, dy)
        if dist < 1e-3:
            return
        ux, uy = dx / dist, dy / dist
        gx, gy = tx - ux * BEHIND, ty - uy * BEHIND       # 꼬리에서 로봇쪽 1.5m
        yaw = math.atan2(ty - gy, tx - gx)                 # 꼬리 바라봄
        if self._goal_xy and math.hypot(gx - self._goal_xy[0], gy - self._goal_xy[1]) < RESEND_DIST:
            return                                          # 거의 같은 목표 → 재전송 안 함
        if not self._nav_ready:
            if not self.client.wait_for_server(timeout_sec=2.0):
                self.get_logger().warn("Nav2 서버 대기...")
                return
            self._nav_ready = True
        self._goal_xy = (gx, gy)
        p = PoseStamped()
        p.header.frame_id = MAP_FRAME
        p.pose.position.x, p.pose.position.y = gx, gy
        p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        g = NavigateToPose.Goal(); g.pose = p
        self.get_logger().info(
            f"★ 비전 꼬리 검출 tail=({tx:.2f},{ty:.2f}) → 후방목표=({gx:.2f},{gy:.2f}) 전송")
        self.client.send_goal_async(g)
        self._goal_sent = True   # 탐색 회전 중지 → Nav2 가 주행 제어


def main():
    rclpy.init()
    try:
        rclpy.spin(CowTailSeek())
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
