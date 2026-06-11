import os

import aiosqlite

# DB 경로: env(DB_PATH) 우선, 없으면 이 파일 옆(로컬 실행도 그대로 동작).
# 도커에선 docker-compose 에서 DB_PATH=/app/cattle_health.db 로 지정.
DB_PATH = os.environ.get(
    "DB_PATH", os.path.join(os.path.dirname(os.path.abspath(__file__)), "cattle_health.db"))



async def get_db():
    async with aiosqlite.connect(DB_PATH) as db:
        db.row_factory = aiosqlite.Row
        yield db

_connection = None

async def get_conn():
    global _connection
    # 연결이 없거나 닫혀있으면 새로 생성
    if _connection is None:
        _connection = await aiosqlite.connect(DB_PATH)
        _connection.row_factory = aiosqlite.Row # 조회 시 딕셔너리처럼 쓰기 위해
    return _connection

async def close_db():
    global _connection
    if _connection:
        await _connection.close()
        _connection = None

async def init_db():
    """서버 시작 시 테이블 생성"""
    async with aiosqlite.connect(DB_PATH) as conn:
        await conn.execute("PRAGMA journal_mode=WAL")

        await conn.execute("""
            CREATE TABLE IF NOT EXISTS detections (
                id        INTEGER PRIMARY KEY AUTOINCREMENT,
                cow_id    TEXT NOT NULL,
                disease   TEXT NOT NULL,
                severity  TEXT NOT NULL,
                timestamp TEXT NOT NULL
            )
        """)

        await conn.execute("""
            CREATE TABLE IF NOT EXISTS cattle (
                id         INTEGER PRIMARY KEY AUTOINCREMENT,
                reg_number TEXT NOT NULL UNIQUE,
                weight     REAL,
                birth_date TEXT,
                status     TEXT DEFAULT '정상'
            )
        """)

        await conn.execute("""
            CREATE TABLE IF NOT EXISTS patrol_course (
                id          INTEGER PRIMARY KEY AUTOINCREMENT,
                course_no   TEXT NOT NULL UNIQUE,
                patrol_time TEXT
            )
        """)

        await conn.execute("""
            CREATE TABLE IF NOT EXISTS barn (
                id              INTEGER PRIMARY KEY AUTOINCREMENT,
                barn_no         TEXT NOT NULL UNIQUE,
                emergency_count INTEGER DEFAULT 0,
                date            TEXT
            )
        """)
        for barn in ['A동', 'B동', 'C동', 'D동']:
            await conn.execute(
                "INSERT OR IGNORE INTO barn (barn_no) VALUES (?)", (barn,)
            )

        # 기존 DB에 컬럼이 없을 경우 안전하게 추가
        for col in [
            "ALTER TABLE cattle ADD COLUMN last_disease TEXT",
            "ALTER TABLE cattle ADD COLUMN last_severity TEXT",
            "ALTER TABLE cattle ADD COLUMN last_detected_at TEXT",
        ]:
            try:
                await conn.execute(col)
            except Exception:
                pass

        for course_no, patrol_time in [('A코스', '09:00'), ('B코스', '13:00'), ('C코스', '17:00')]:
            await conn.execute(
                "INSERT OR IGNORE INTO patrol_course (course_no, patrol_time) VALUES (?,?)",
                (course_no, patrol_time)
            )

        await conn.commit()
