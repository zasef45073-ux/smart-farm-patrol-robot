import asyncio
from fastapi import APIRouter, WebSocket, WebSocketDisconnect
from pydantic import BaseModel
from typing import List

camera_router = APIRouter()

connected_clients: List[WebSocket] = []

_latest_frame: str | None = None
_frame_lock = asyncio.Lock()


class CameraIn(BaseModel):
    image: str


async def set_latest_frame(image_data: str):
    global _latest_frame
    async with _frame_lock:
        _latest_frame = image_data


async def get_latest_frame() -> str | None:
    async with _frame_lock:
        return _latest_frame


@camera_router.websocket("/ws/camera")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    connected_clients.append(websocket)
    try:
        while True:
            try:
                await asyncio.wait_for(websocket.receive_text(), timeout=25.0)
            except asyncio.TimeoutError:
                await websocket.send_text("ping")
    except (WebSocketDisconnect, Exception):
        if websocket in connected_clients:
            connected_clients.remove(websocket)


@camera_router.post("/api/camera")
async def receive_camera(data: CameraIn):
    await set_latest_frame(data.image)
    dead = []
    for client in connected_clients:
        try:
            await client.send_text(data.image)
        except Exception:
            dead.append(client)
    for client in dead:
        connected_clients.remove(client)
    return {"status": "ok"}


@camera_router.get("/api/camera/latest")
async def get_camera():
    return {"image": await get_latest_frame()}
