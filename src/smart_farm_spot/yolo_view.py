#!/usr/bin/env python3
# Copyright (c) 2024, Smart Farm Patrol Project - 꼬마 로봇 두리
# SPDX-License-Identifier: Apache-2.0
"""
yolo_view.py
============
Isaac 전방 카메라(/front_stereo_camera/left/image_raw)를 구독해 YOLO(best.pt, pose)
로 소를 검출하고, 박스/키포인트를 그려 시각화한다.

  - /yolo/annotated  (sensor_msgs/Image)         : 검출 결과 오버레이 (rqt_image_view 로 보기)
  - /cow/lameness    (std_msgs/Float32MultiArray) : 소별 파행(보행이상) 비대칭 지표(0~1, 클수록 의심)
  - /tmp/yolo_latest.jpg                          : 최신 프레임 저장(확인용)
  - depth(/left/depth) + camera_info 로 검출된 소까지의 거리(중앙 깊이) 로그

[실행]
  ROS_DOMAIN_ID=153 python3 yolo_view.py
  rqt_image_view /yolo/annotated      # 결과 보기
"""
import os

import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
from ultralytics import YOLO

_PKG = os.path.dirname(os.path.abspath(__file__))
WEIGHTS = os.path.join(_PKG, "assets", "yolo", "best.pt")
CONF = 0.50

# 파행(lameness) 판정용 — best.pt 는 'cow' 1클래스 + 14키포인트만 학습돼 파행 라벨이 없으므로,
# 키포인트의 좌우(bbox 세로 중심선 기준) 대칭성으로 보행이상을 정량화한다.
# 절름발이/비정상 자세는 좌우 사지 키포인트 배치가 비대칭해지므로 지표가 커진다.
# 키포인트 의미순서(어느 인덱스가 왼/오른 다리인지)에 의존하지 않도록 미러-매칭 방식 사용.
KP_CONF = 0.30        # 키포인트 신뢰도 하한(이상만 대칭계산에 사용)
KP_MIN_PTS = 4        # 대칭 지표를 계산할 최소 신뢰 키포인트 수
LAMENESS_WARN = 0.18  # 이 값을 넘으면 파행 의심 경고(휴리스틱)

TOPIC_RGB   = "/spot_cam/rgb"
TOPIC_DEPTH = "/spot_cam/depth"
TOPIC_INFO  = "/spot_cam/camera_info"


class YoloView(Node):
    def __init__(self):
        super().__init__("yolo_view")
        self.get_logger().info(f"YOLO 로드: {WEIGHTS}")
        self.model = YOLO(WEIGHTS)
        self.bridge = CvBridge()
        self._depth = None
        self._cam_ready = False

        from rclpy.qos import qos_profile_sensor_data
        _q = qos_profile_sensor_data   # best_effort (Isaac 이미지 발행과 일치)
        self.pub = self.create_publisher(Image, "/yolo/annotated", 5)
        self.pub_lame = self.create_publisher(Float32MultiArray, "/cow/lameness", 5)
        self.create_subscription(Image, TOPIC_RGB, self.cb_rgb, _q)
        self.create_subscription(Image, TOPIC_DEPTH, self.cb_depth, _q)
        self.create_subscription(CameraInfo, TOPIC_INFO, self.cb_info, _q)

        self.n = 0
        self.get_logger().info("=" * 50)
        self.get_logger().info("YOLO View 시작 — 전방 카메라에서 소 검출 + 시각화")
        self.get_logger().info(f"  결과: /yolo/annotated  (rqt_image_view 로 확인)")
        self.get_logger().info("=" * 50)

    def cb_info(self, msg: CameraInfo):
        if not self._cam_ready:
            self._cam_ready = True
            self.get_logger().info(f"카메라: {msg.width}x{msg.height}")

    def cb_depth(self, msg: Image):
        try:
            self._depth = self.bridge.imgmsg_to_cv2(msg, "32FC1")
        except Exception as e:
            self.get_logger().warn(f"depth 변환 실패: {e}")

    def cb_rgb(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().warn(f"rgb 변환 실패: {e}")
            return

        results = self.model(frame, conf=CONF, verbose=False)
        r = results[0]
        annotated = r.plot()          # 박스 + 키포인트 + 라벨 오버레이
        ndet = 0 if r.boxes is None else len(r.boxes)

        # 키포인트(있으면) → 소별 파행 비대칭 지표 산출/발행 준비
        kp_xy, kp_conf = self._extract_keypoints(r)
        lame_scores = []

        # 검출된 소 중심 깊이(거리) + 파행 지표 표시
        if ndet and r.boxes is not None:
            for i, b in enumerate(r.boxes):
                x1, y1, x2, y2 = [int(v) for v in b.xyxy[0].tolist()]
                cu, cv = (x1 + x2) // 2, (y1 + y2) // 2

                if self._depth is not None:
                    dist = self._sample_depth(cu, cv, annotated.shape)
                    if dist:
                        cv2.putText(annotated, f"{dist:.2f}m", (x1, max(0, y1 - 8)),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

                score = self._lameness_score(kp_xy, kp_conf, i, (x1, y1, x2, y2))
                lame_scores.append(score)
                if score >= 0.0:
                    warn = score >= LAMENESS_WARN
                    color = (0, 0, 255) if warn else (0, 200, 255)
                    tag = "LAME?" if warn else "ok"
                    cv2.putText(annotated, f"limp {score:.2f} {tag}",
                                (x1, min(annotated.shape[0] - 4, y2 + 22)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        # /cow/lameness 발행 (소별 지표; 신뢰 키포인트 부족 시 -1.0)
        self.pub_lame.publish(Float32MultiArray(data=[float(s) for s in lame_scores]))

        out = self.bridge.cv2_to_imgmsg(annotated, "bgr8")
        out.header = msg.header
        self.pub.publish(out)

        self.n += 1
        if self.n % 15 == 0:
            cv2.imwrite("/tmp/yolo_latest.jpg", annotated)
            valid = [s for s in lame_scores if s >= 0.0]
            lame_txt = f", 파행지표 max={max(valid):.2f}" if valid else ""
            self.get_logger().info(
                f"[{self.n}] 검출 {ndet}마리{lame_txt} → /yolo/annotated, /cow/lameness")

    def _sample_depth(self, u, v, ann_shape):
        if self._depth is None:
            return None
        dh, dw = self._depth.shape[:2]
        ah, aw = ann_shape[:2]
        du = int(u * dw / aw)
        dv = int(v * dh / ah)
        if 0 <= dv < dh and 0 <= du < dw:
            d = float(self._depth[dv, du])
            if np.isfinite(d) and 0.1 < d < 40.0:
                return d
        return None

    @staticmethod
    def _extract_keypoints(r):
        """YOLO 결과에서 (xy, conf) 배열을 꺼낸다. pose 모델이 아니면 (None, None)."""
        kp = getattr(r, "keypoints", None)
        if kp is None or kp.xy is None:
            return None, None
        try:
            xy = kp.xy.cpu().numpy()                       # (ndet, nkpt, 2)
            conf = (kp.conf.cpu().numpy()                  # (ndet, nkpt)
                    if kp.conf is not None else
                    np.ones(xy.shape[:2], dtype=np.float32))
            return xy, conf
        except Exception:
            return None, None

    def _lameness_score(self, kp_xy, kp_conf, i, bbox):
        """소 한 마리의 파행(보행이상) 비대칭 지표(0~1).

        파행은 좌우 사지의 자세가 비대칭해지는 특징이 있다. best.pt 에는 좌/우 키포인트
        라벨 정보가 없으므로, bbox 세로 중심선 기준으로 키포인트 구름을 좌우 미러링한 뒤
        원본과 가장 가까운 점까지의 거리를 bbox 대각선으로 정규화해 평균한다.
        좌우 대칭이면 0 에 가깝고, 한쪽으로 쏠린(절름) 자세면 값이 커진다.
        신뢰 키포인트가 부족하면 -1.0(판정 불가)을 반환한다.
        """
        if kp_xy is None or i >= len(kp_xy):
            return -1.0
        pts = kp_xy[i]
        conf = kp_conf[i]
        good = (conf >= KP_CONF) & np.isfinite(pts).all(axis=1) & (pts != 0).any(axis=1)
        pts = pts[good]
        if len(pts) < KP_MIN_PTS:
            return -1.0

        x1, y1, x2, y2 = bbox
        diag = float(np.hypot(max(1, x2 - x1), max(1, y2 - y1)))
        cx = float(pts[:, 0].mean())                       # 세로 중심선(키포인트 중심)
        mirrored = pts.copy()
        mirrored[:, 0] = 2.0 * cx - mirrored[:, 0]         # 중심선 기준 좌우 반전

        # 각 원본 점 → 가장 가까운 미러 점까지의 거리(자기 자신 포함 가능, 대칭이면 0)
        d = np.linalg.norm(pts[:, None, :] - mirrored[None, :, :], axis=2)
        nn = d.min(axis=1)
        return float(np.clip(nn.mean() / diag, 0.0, 1.0))


def main(args=None):
    rclpy.init(args=args)
    try:
        rclpy.spin(YoloView())
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
