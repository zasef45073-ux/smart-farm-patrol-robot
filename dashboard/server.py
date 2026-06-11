import asyncio
import json
import os
from contextlib import asynccontextmanager
from datetime import datetime

from dotenv import load_dotenv
load_dotenv()

import aiosqlite
import paho.mqtt.client as mqtt
from apscheduler.schedulers.asyncio import AsyncIOScheduler
from apscheduler.triggers.cron import CronTrigger
from apscheduler.triggers.interval import IntervalTrigger
from fastapi import FastAPI, Form, Request
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import RedirectResponse
from fastapi.templating import Jinja2Templates
from starlette.middleware.sessions import SessionMiddleware

from database import DB_PATH, close_db, init_db
from routes.camera import camera_router
from routes.detection import detection_router
from routes.edr import edr_router
from routes.robot import robot_router, set_status

MQTT_BROKER  = "mosquitto"
MQTT_PORT    = 1883
PATROL_TOPIC = "cattle/patrol/command"

LOGIN_USERNAME = os.environ["LOGIN_USERNAME"]
LOGIN_PASSWORD = os.environ["LOGIN_PASSWORD"]
SESSION_SECRET = os.environ["SESSION_SECRET"]

scheduler = AsyncIOScheduler()
_mqtt_client: mqtt.Client | None = None


def _get_mqtt() -> mqtt.Client:
    global _mqtt_client
    if _mqtt_client is None:
        client = mqtt.Client()
        client.connect(MQTT_BROKER, MQTT_PORT, keepalive=60)
        client.loop_start()
        _mqtt_client = client
    return _mqtt_client


async def _run_patrol(course_no: str):
    now_str = datetime.now().strftime("%H:%M")
    payload = json.dumps({"command": "start"})
    _get_mqtt().publish(PATROL_TOPIC, payload)
    set_status("patrolling", course_no)
    print(f"[APScheduler] {now_str} → 순찰 명령 발행")
    await asyncio.sleep(60)
    set_status("idle", None)
    print(f"[APScheduler] {course_no} 순찰 완료 → 대기 중")


async def _load_schedules():
    async with aiosqlite.connect(DB_PATH) as conn:
        conn.row_factory = aiosqlite.Row
        async with conn.execute(
            "SELECT course_no, patrol_time FROM patrol_course WHERE patrol_time IS NOT NULL"
        ) as c:
            rows = await c.fetchall()

    for row in rows:
        course_no   = row["course_no"]
        patrol_time = row["patrol_time"]
        scheduler.add_job(
            _run_patrol,
            IntervalTrigger(minutes=2),
            args=[course_no],
            id=f"patrol_{course_no}",
            replace_existing=True,
        )
        print(f"[APScheduler] 등록: {course_no} → 5분마다 (원래 설정: {patrol_time})")


@asynccontextmanager
async def lifespan(app: FastAPI):
    await init_db()
    await _load_schedules()
    scheduler.start()
    print("[APScheduler] 스케줄러 시작됨")
    yield
    scheduler.shutdown()
    if _mqtt_client:
        _mqtt_client.loop_stop()
        _mqtt_client.disconnect()
    await close_db()


app = FastAPI(
    title="축사 자율순찰 로봇 API",
    description="질병 감지, 카메라 스트리밍, EDR 관리 API",
    version="4.0.0",
    lifespan=lifespan,
)

app.add_middleware(SessionMiddleware, secret_key=SESSION_SECRET)
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

app.include_router(detection_router)
app.include_router(camera_router)
app.include_router(robot_router)
app.include_router(edr_router)

templates = Jinja2Templates(directory="templates")


def _is_logged_in(request: Request) -> bool:
    return request.session.get("user") == LOGIN_USERNAME


@app.get("/login")
async def login_page(request: Request):
    if _is_logged_in(request):
        return RedirectResponse("/", status_code=302)
    return templates.TemplateResponse(request=request, name="login.html", context={"error": None})


@app.post("/login")
async def login(request: Request, username: str = Form(...), password: str = Form(...)):
    if username == LOGIN_USERNAME and password == LOGIN_PASSWORD:
        request.session["user"] = username
        return RedirectResponse("/", status_code=302)
    return templates.TemplateResponse(
        request=request, name="login.html", context={"error": "아이디 또는 비밀번호가 올바르지 않습니다"}
    )


@app.get("/logout")
async def logout(request: Request):
    request.session.clear()
    return RedirectResponse("/login", status_code=302)


@app.get("/")
async def dashboard(request: Request):
    if not _is_logged_in(request):
        return RedirectResponse("/login", status_code=302)
    return templates.TemplateResponse(request=request, name="dashboard.html")


@app.get("/camera")
async def camera_page(request: Request):
    if not _is_logged_in(request):
        return RedirectResponse("/login", status_code=302)
    return templates.TemplateResponse(request=request, name="camera.html")


@app.get("/edr")
async def edr_page(request: Request):
    if not _is_logged_in(request):
        return RedirectResponse("/login", status_code=302)
    return templates.TemplateResponse(request=request, name="edr.html")


if __name__ == "__main__":
    import uvicorn
    uvicorn.run("server:app", host="0.0.0.0", port=5000, reload=True)
