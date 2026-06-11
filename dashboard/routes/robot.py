from fastapi import APIRouter, HTTPException
from pydantic import BaseModel

robot_router = APIRouter()

_robot_status:   str       = "idle"
_current_course: str | None = None


class RobotStatusIn(BaseModel):
    status: str
    course: str | None = None


class PatrolIn(BaseModel):
    course_no: str


def set_status(status: str, course: str | None = None):
    """로봇 상태 업데이트 (HTTP·ROS2 양쪽에서 호출)"""
    global _robot_status, _current_course
    _robot_status   = status
    _current_course = course


@robot_router.get("/api/robot/status")
async def get_robot_status():
    """로봇 순찰 상태 및 현재 코스 조회"""
    return {"status": _robot_status, "course": _current_course}


@robot_router.post("/api/robot/status")
async def post_robot_status(data: RobotStatusIn):
    """로봇 상태 업데이트 (ROS2에서 HTTP로 호출)"""
    set_status(data.status, data.course)
    return {"status": _robot_status, "course": _current_course}



@robot_router.post("/api/robot/estop")
async def emergency_stop():
    """비상 정지 — 모든 동작 즉시 중단"""
    try:
        from ros2_bridge import send_estop_command
        send_estop_command()
    except RuntimeError:
        pass
    set_status("estop", None)
    return {"status": "estop"}
