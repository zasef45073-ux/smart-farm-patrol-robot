import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import asyncio
import aiohttp
import base64
import threading
import time
from datetime import datetime
from detect_mastitis import detect_mastitis_array

import os
# 카메라 토픽: env(CAMERA_TOPIC)로 로봇 토픽에 맞춤(브리지).
#   smart_farm_spot 로봇이면 예: CAMERA_TOPIC=/spot_cam/rgb (또는 /spot/arm/rgb/image_raw)
TOPIC      = os.environ.get("CAMERA_TOPIC", "/rgb_camera/image_raw")
SERVER_URL = os.environ.get("CAMERA_API", "http://localhost:5000/api/camera")
SAVE_URL   = os.environ.get("SAVE_API", "http://localhost:5000/api/save")
SEND_INTERVAL   = 0.1   # 초당 10프레임 네트워크 상황에 따라 fps가 버거우면 SEND_INTERVAL을 0.2 (5fps)
DETECT_EVERY_N  = 30    # N프레임마다 유방염 검사 (10fps 기준 약 3초)
MASTITIS_THRESH = 0.5   # 빨간색 비율 임계치 (%)


class CameraStreamer(Node):
    def __init__(self, loop: asyncio.AbstractEventLoop):
        super().__init__('camera_streamer')
        self.bridge = CvBridge()
        self.loop = loop
        self._session: aiohttp.ClientSession | None = None
        self._last_sent = 0.0
        self._frame_count = 0
        self.subscription = self.create_subscription(
            Image, TOPIC, self.image_callback, 10
        )
        self.get_logger().info(f"카메라 토픽 구독 시작: {TOPIC}")

    async def open_session(self):
        self._session = aiohttp.ClientSession()

    async def close_session(self):
        if self._session:
            await self._session.close()

    async def _send(self, img_base64: str):
        if self._session is None:
            return
        try:
            timeout = aiohttp.ClientTimeout(total=1)
            async with self._session.post(
                SERVER_URL, json={"image": img_base64}, timeout=timeout
            ):
                pass
        except Exception as e:
            self.get_logger().warn(f"카메라 전송 실패: {e}")

    async def _send_detection(self, payload: dict):
        if self._session is None:
            return
        try:
            timeout = aiohttp.ClientTimeout(total=2)
            async with self._session.post(SAVE_URL, json=payload, timeout=timeout):
                pass
        except Exception as e:
            self.get_logger().warn(f"감지 전송 실패: {e}")

    def image_callback(self, msg):
        now = time.monotonic()
        if now - self._last_sent < SEND_INTERVAL:
            return
        self._last_sent = now

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            if len(cv_image.shape) == 2:
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2BGR)
            elif cv_image.shape[2] == 4:
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGBA2BGR)
            elif msg.encoding in ('rgb8', 'rgb16'):
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
            _, buffer = cv2.imencode('.jpg', cv_image, [cv2.IMWRITE_JPEG_QUALITY, 70])
            img_base64 = base64.b64encode(buffer).decode('utf-8')
            asyncio.run_coroutine_threadsafe(self._send(img_base64), self.loop)

            self._frame_count += 1
            if self._frame_count % DETECT_EVERY_N == 0:
                result = detect_mastitis_array(cv_image, MASTITIS_THRESH)
                if result["detected"]:
                    ts = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
                    payload = {
                        "cow_id":    "카메라 감지",
                        "disease":   "유방염 의심",
                        "severity":  "위험",
                        "timestamp": ts,
                    }
                    asyncio.run_coroutine_threadsafe(self._send_detection(payload), self.loop)
                    self.get_logger().warn(
                        f"🔴 유방염 감지 (빨간비율 {result['ratio']:.2f}%) → 서버 전송"
                    )
        except Exception as e:
            self.get_logger().warn(f"이미지 변환 실패: {e}")


def main():
    loop = asyncio.new_event_loop()
    threading.Thread(target=loop.run_forever, daemon=True).start()

    rclpy.init()
    node = CameraStreamer(loop)
    asyncio.run_coroutine_threadsafe(node.open_session(), loop).result()

    try:
        rclpy.spin(node)
    finally:
        asyncio.run_coroutine_threadsafe(node.close_session(), loop).result()
        node.destroy_node()
        rclpy.shutdown()
        loop.call_soon_threadsafe(loop.stop)


if __name__ == '__main__':
    main()
