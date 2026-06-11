"""감지(detection) 라우트 — 질병 감지 저장/조회/요약 + SSE 실시간 스트림."""
import asyncio
import json
from datetime import datetime

import aiosqlite
from fastapi import APIRouter
from fastapi.responses import StreamingResponse
from pydantic import BaseModel

from database import DB_PATH

detection_router = APIRouter()

# ── SSE 구독자(메모리 브로드캐스터) ──
_subscribers: list[asyncio.Queue] = []


def _broadcast(event: dict):
    for q in list(_subscribers):
        try:
            q.put_nowait(event)
        except Exception:
            pass


class DetectionIn(BaseModel):
    cow_id: str
    disease: str
    severity: str
    timestamp: str | None = None


@detection_router.post("/api/save")
async def save_detection(data: DetectionIn):
    ts = data.timestamp or datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    async with aiosqlite.connect(DB_PATH) as db:
        await db.execute(
            "INSERT INTO detections (cow_id, disease, severity, timestamp) VALUES (?,?,?,?)",
            (data.cow_id, data.disease, data.severity, ts),
        )
        # 해당 소의 최근 상태 갱신(존재할 때만)
        await db.execute(
            "UPDATE cattle SET last_disease=?, last_severity=?, last_detected_at=?, status=? "
            "WHERE reg_number=?",
            (data.disease, data.severity, ts,
             "위험" if data.severity == "위험" else "정상", data.cow_id),
        )
        await db.commit()
    event = {"cow_id": data.cow_id, "disease": data.disease,
             "severity": data.severity, "timestamp": ts}
    _broadcast(event)
    return {"status": "ok", **event}


@detection_router.get("/api/detections")
async def list_detections(limit: int = 50):
    async with aiosqlite.connect(DB_PATH) as db:
        db.row_factory = aiosqlite.Row
        async with db.execute(
            "SELECT id, cow_id, disease, severity, timestamp FROM detections "
            "ORDER BY id DESC LIMIT ?", (limit,)
        ) as c:
            rows = await c.fetchall()
    return [dict(r) for r in rows]


@detection_router.delete("/api/detections")
async def clear_detections():
    async with aiosqlite.connect(DB_PATH) as db:
        await db.execute("DELETE FROM detections")
        await db.execute(
            "UPDATE cattle SET last_disease=NULL, last_severity=NULL, "
            "last_detected_at=NULL, status='정상'")
        await db.commit()
    return {"status": "cleared"}


@detection_router.get("/api/summary")
async def summary():
    async with aiosqlite.connect(DB_PATH) as db:
        db.row_factory = aiosqlite.Row
        async with db.execute(
            "SELECT severity, COUNT(*) AS n FROM detections GROUP BY severity"
        ) as c:
            rows = await c.fetchall()
    counts = {r["severity"]: r["n"] for r in rows}
    danger = counts.get("위험", 0)
    normal = sum(n for s, n in counts.items() if s != "위험")
    return {"danger": danger, "normal": normal}


@detection_router.get("/api/stream")
async def stream():
    """SSE — 새 감지 이벤트를 실시간 푸시."""
    q: asyncio.Queue = asyncio.Queue()
    _subscribers.append(q)

    async def gen():
        try:
            while True:
                event = await q.get()
                yield f"data: {json.dumps(event, ensure_ascii=False)}\n\n"
        finally:
            if q in _subscribers:
                _subscribers.remove(q)

    return StreamingResponse(gen(), media_type="text/event-stream")
