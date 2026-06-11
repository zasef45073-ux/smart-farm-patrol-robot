import threading
import asyncio
import json

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False

PATROL_CMD_TOPIC   = "/patrol/command"
ROBOT_STATUS_TOPIC = "/robot/status"

_node: "ROS2Bridge | None" = None


class ROS2Bridge(Node):
    def __init__(self, loop: asyncio.AbstractEventLoop):
        super().__init__('fastapi_ros2_bridge')
        self._loop = loop
        self._pub  = self.create_publisher(String, PATROL_CMD_TOPIC, 10)
        self.create_subscription(String, ROBOT_STATUS_TOPIC, self._on_status, 10)
        self.get_logger().info(
            f"ROS2 브리지 준비 | 명령:{PATROL_CMD_TOPIC}  상태:{ROBOT_STATUS_TOPIC}"
        )

    def publish_patrol(self, course_no: str):
        msg = String()
        msg.data = json.dumps({"command": "start", "course": course_no})
        self._pub.publish(msg)
        self.get_logger().info(f"순찰 명령 발행 → {course_no}")

    def publish_estop(self):
        msg = String()
        msg.data = json.dumps({"command": "estop"})
        self._pub.publish(msg)
        self.get_logger().info("비상 정지 명령 발행")

    def _on_status(self, msg: String):
        try:
            data = json.loads(msg.data)
            asyncio.run_coroutine_threadsafe(_apply_status(data), self._loop)
        except Exception as e:
            self.get_logger().warn(f"상태 메시지 파싱 실패: {e}")


async def _apply_status(data: dict):
    from routes.robot import set_status
    set_status(data.get("status", "idle"), data.get("course"))


def _spin(loop: asyncio.AbstractEventLoop):
    global _node
    try:
        rclpy.init()
        _node = ROS2Bridge(loop)
        rclpy.spin(_node)
    except Exception as e:
        print(f"[ROS2] 초기화 실패: {e}")
    finally:
        if _node:
            _node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


def start():
    """ROS2 브리지를 백그라운드 스레드에서 시작"""
    loop = asyncio.get_running_loop()
    threading.Thread(target=_spin, args=(loop,), daemon=True).start()


def send_patrol_command(course_no: str):
    if _node is None:
        raise RuntimeError("ROS2 브리지가 초기화되지 않았습니다")
    _node.publish_patrol(course_no)


def send_estop_command():
    if _node is None:
        raise RuntimeError("ROS2 브리지가 초기화되지 않았습니다")
    _node.publish_estop()
