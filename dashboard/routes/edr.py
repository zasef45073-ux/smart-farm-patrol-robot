"""EDR 라우트 — 소 개체 / 순찰 코스 / 축사 동 CRUD."""
import aiosqlite
from fastapi import APIRouter, HTTPException
from pydantic import BaseModel

from database import DB_PATH

edr_router = APIRouter()


# ══ 소 개체 ══
class CattleIn(BaseModel):
    reg_number: str
    weight: float | None = None
    birth_date: str | None = None
    status: str = "정상"


@edr_router.get("/api/edr/cattle")
async def list_cattle():
    async with aiosqlite.connect(DB_PATH) as db:
        db.row_factory = aiosqlite.Row
        async with db.execute(
            "SELECT id, reg_number, weight, birth_date, status, "
            "last_disease, last_severity, last_detected_at FROM cattle ORDER BY id"
        ) as c:
            return [dict(r) for r in await c.fetchall()]


@edr_router.post("/api/edr/cattle")
async def add_cattle(c: CattleIn):
    async with aiosqlite.connect(DB_PATH) as db:
        try:
            cur = await db.execute(
                "INSERT INTO cattle (reg_number, weight, birth_date, status) VALUES (?,?,?,?)",
                (c.reg_number, c.weight, c.birth_date, c.status))
            await db.commit()
        except aiosqlite.IntegrityError:
            raise HTTPException(400, f"이미 존재하는 개체번호: {c.reg_number}")
    return {"id": cur.lastrowid, **c.model_dump()}


@edr_router.put("/api/edr/cattle/{cid}")
async def update_cattle(cid: int, c: CattleIn):
    async with aiosqlite.connect(DB_PATH) as db:
        await db.execute(
            "UPDATE cattle SET reg_number=?, weight=?, birth_date=?, status=? WHERE id=?",
            (c.reg_number, c.weight, c.birth_date, c.status, cid))
        await db.commit()
    return {"id": cid, **c.model_dump()}


@edr_router.delete("/api/edr/cattle/{cid}")
async def delete_cattle(cid: int):
    async with aiosqlite.connect(DB_PATH) as db:
        await db.execute("DELETE FROM cattle WHERE id=?", (cid,))
        await db.commit()
    return {"status": "deleted", "id": cid}


# ══ 순찰 코스 ══
class CourseIn(BaseModel):
    course_no: str
    patrol_time: str | None = None


@edr_router.get("/api/edr/course")
async def list_course():
    async with aiosqlite.connect(DB_PATH) as db:
        db.row_factory = aiosqlite.Row
        async with db.execute(
            "SELECT id, course_no, patrol_time FROM patrol_course ORDER BY id") as c:
            return [dict(r) for r in await c.fetchall()]


@edr_router.post("/api/edr/course")
async def add_course(c: CourseIn):
    async with aiosqlite.connect(DB_PATH) as db:
        try:
            cur = await db.execute(
                "INSERT INTO patrol_course (course_no, patrol_time) VALUES (?,?)",
                (c.course_no, c.patrol_time))
            await db.commit()
        except aiosqlite.IntegrityError:
            raise HTTPException(400, f"이미 존재하는 코스: {c.course_no}")
    return {"id": cur.lastrowid, **c.model_dump()}


@edr_router.put("/api/edr/course/{cid}")
async def update_course(cid: int, c: CourseIn):
    async with aiosqlite.connect(DB_PATH) as db:
        await db.execute(
            "UPDATE patrol_course SET course_no=?, patrol_time=? WHERE id=?",
            (c.course_no, c.patrol_time, cid))
        await db.commit()
    return {"id": cid, **c.model_dump()}


@edr_router.delete("/api/edr/course/{cid}")
async def delete_course(cid: int):
    async with aiosqlite.connect(DB_PATH) as db:
        await db.execute("DELETE FROM patrol_course WHERE id=?", (cid,))
        await db.commit()
    return {"status": "deleted", "id": cid}


# ══ 축사 동 ══
class BarnIn(BaseModel):
    barn_no: str
    emergency_count: int = 0
    date: str | None = None


@edr_router.get("/api/edr/barn")
async def list_barn():
    async with aiosqlite.connect(DB_PATH) as db:
        db.row_factory = aiosqlite.Row
        async with db.execute(
            "SELECT id, barn_no, emergency_count, date FROM barn ORDER BY id") as c:
            return [dict(r) for r in await c.fetchall()]


@edr_router.post("/api/edr/barn")
async def add_barn(b: BarnIn):
    async with aiosqlite.connect(DB_PATH) as db:
        try:
            cur = await db.execute(
                "INSERT INTO barn (barn_no, emergency_count, date) VALUES (?,?,?)",
                (b.barn_no, b.emergency_count, b.date))
            await db.commit()
        except aiosqlite.IntegrityError:
            raise HTTPException(400, f"이미 존재하는 동: {b.barn_no}")
    return {"id": cur.lastrowid, **b.model_dump()}


@edr_router.put("/api/edr/barn/{bid}")
async def update_barn(bid: int, b: BarnIn):
    async with aiosqlite.connect(DB_PATH) as db:
        await db.execute(
            "UPDATE barn SET barn_no=?, emergency_count=?, date=? WHERE id=?",
            (b.barn_no, b.emergency_count, b.date, bid))
        await db.commit()
    return {"id": bid, **b.model_dump()}


@edr_router.delete("/api/edr/barn/{bid}")
async def delete_barn(bid: int):
    async with aiosqlite.connect(DB_PATH) as db:
        await db.execute("DELETE FROM barn WHERE id=?", (bid,))
        await db.commit()
    return {"status": "deleted", "id": bid}
