# Copyright (c) 2024, Smart Farm Patrol Project - 꼬마 로봇 두리
# SPDX-License-Identifier: Apache-2.0
"""
thermal_processor.py
====================
Isaac Sim RGB 영상을 열화상으로 변환하고,
유방염 열점(Hotspot) Bounding Box를 추출하는 유틸리티.

Isaac Sim 네이티브 열화상 미지원 대응 전략:
  - 소 유방 머티리얼에 Emission(40~42°C 발광) 설정
  - RGB 카메라로 수신 → OpenCV JET colormap 후처리 → 열화상 모사

ROS 2 연동:
  구독: /spot/arm/thermal/image_raw  (sensor_msgs/Image, rgb8)
  발행: /spot/arm/thermal/colormap   (sensor_msgs/Image, bgr8)
  발행: /spot/arm/thermal/hotspot    (vision_msgs/Detection2D)

실행:
  python3 thermal_processor.py
"""

import cv2
import numpy as np


# ══════════════════════════════════════════════════════════════════════
# 핵심 처리 함수
# ══════════════════════════════════════════════════════════════════════

def apply_fake_thermal(rgb_image: np.ndarray) -> np.ndarray:
    """
    Isaac Sim RGB 이미지를 열화상처럼 변환.

    소 유방 Emission 머티리얼(고발광 픽셀) → 붉은 열점으로 표현.

    Args:
        rgb_image: (H, W, 3) uint8 RGB 이미지

    Returns:
        thermal_vis: (H, W, 3) uint8 BGR 열화상 이미지
    """
    # 1. 그레이스케일 변환
    gray = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2GRAY)

    # 2. JET colormap 적용 (파랑=차가움 → 빨강=뜨거움)
    thermal_vis = cv2.applyColorMap(gray, cv2.COLORMAP_JET)

    # 3. Emission 픽셀(밝기 > 200) → 40°C 이상으로 간주, 순백/주황으로 강조
    hotspot_mask = gray > 200
    thermal_vis[hotspot_mask] = [0, 128, 255]   # 주황 (BGR)

    # 4. 매우 높은 발광(밝기 > 240) → 42°C 이상, 순백으로 표시
    extreme_mask = gray > 240
    thermal_vis[extreme_mask] = [255, 255, 255]  # 흰색

    return thermal_vis


def get_hotspot_bbox(
    thermal_img: np.ndarray,
    min_area: int = 500,
) -> dict | None:
    """
    열화상 이미지에서 유방염 열점(Hotspot) Bounding Box 추출.

    Args:
        thermal_img: apply_fake_thermal() 출력 BGR 이미지
        min_area:    최소 열점 면적 (픽셀²) - 노이즈 제거

    Returns:
        dict: {"x": int, "y": int, "w": int, "h": int,
               "cx": float, "cy": float, "area": float}
        None: 열점 미검출
    """
    # 주황/흰색 마스킹 (BGR)
    lower_orange = np.array([0, 100, 200])
    upper_orange = np.array([30, 180, 255])
    mask_orange = cv2.inRange(thermal_img, lower_orange, upper_orange)

    lower_white = np.array([200, 200, 200])
    upper_white = np.array([255, 255, 255])
    mask_white = cv2.inRange(thermal_img, lower_white, upper_white)

    hotspot_mask = cv2.bitwise_or(mask_orange, mask_white)

    # 모폴로지: 잡음 제거
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    hotspot_mask = cv2.morphologyEx(hotspot_mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(
        hotspot_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    if not contours:
        return None

    c = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(c)
    if area < min_area:
        return None

    x, y, w, h = cv2.boundingRect(c)
    return {
        "x": x, "y": y, "w": w, "h": h,
        "cx": x + w / 2,
        "cy": y + h / 2,
        "area": area,
    }


def draw_hotspot_overlay(
    frame: np.ndarray,
    bbox: dict,
    label: str = "MASTITIS",
    color: tuple = (0, 0, 255),
) -> np.ndarray:
    """
    열화상 이미지에 열점 탐지 결과를 시각화.

    Args:
        frame: BGR 이미지
        bbox:  get_hotspot_bbox() 반환값
        label: 표시할 레이블
        color: 박스 색상 (BGR)

    Returns:
        overlay: 시각화된 BGR 이미지
    """
    overlay = frame.copy()
    x, y, w, h = bbox["x"], bbox["y"], bbox["w"], bbox["h"]

    # 열점 박스
    cv2.rectangle(overlay, (x, y), (x + w, y + h), color, 2)

    # 레이블
    label_text = f"{label} ({bbox['area']:.0f}px²)"
    cv2.putText(
        overlay, label_text, (x, y - 8),
        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2
    )

    # 중심점 십자선
    cx, cy = int(bbox["cx"]), int(bbox["cy"])
    cv2.drawMarker(overlay, (cx, cy), color, cv2.MARKER_CROSS, 15, 2)

    return overlay


# ══════════════════════════════════════════════════════════════════════
# ROS 2 노드 (선택적 실행)
# ══════════════════════════════════════════════════════════════════════

def main():
    """
    ROS 2 열화상 처리 노드.
    /spot/arm/thermal/image_raw 구독 → colormap + hotspot 발행.
    """
    try:
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import Image
        from cv_bridge import CvBridge
    except ImportError:
        print("[ERROR] rclpy 또는 cv_bridge 가 없습니다.")
        print("       ROS 2 Humble 환경에서 실행하세요.")
        return

    class ThermalNode(Node):
        def __init__(self):
            super().__init__("thermal_processor")
            self.bridge = CvBridge()
            self.sub = self.create_subscription(
                Image,
                "/spot/arm/thermal/image_raw",
                self.callback,
                10,
            )
            self.pub_colormap = self.create_publisher(
                Image, "/spot/arm/thermal/colormap", 10
            )
            self.get_logger().info("ThermalProcessor 노드 시작")

        def callback(self, msg: Image):
            rgb = self.bridge.imgmsg_to_cv2(msg, "rgb8")

            # 열화상 변환
            thermal = apply_fake_thermal(rgb)

            # 열점 탐지
            bbox = get_hotspot_bbox(thermal)
            if bbox:
                thermal = draw_hotspot_overlay(thermal, bbox)
                self.get_logger().info(
                    f"유방염 열점 감지: cx={bbox['cx']:.1f}, cy={bbox['cy']:.1f}"
                )

            # 발행
            out_msg = self.bridge.cv2_to_imgmsg(thermal, "bgr8")
            out_msg.header = msg.header
            self.pub_colormap.publish(out_msg)

    rclpy.init()
    node = ThermalNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
