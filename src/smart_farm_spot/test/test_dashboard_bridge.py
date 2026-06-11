# Copyright (c) 2024, Smart Farm Patrol Project - 꼬마 로봇 두리
# SPDX-License-Identifier: Apache-2.0
"""대시보드 브리지 명령 파싱/상태 빌드 테스트 (rclpy 불요 — import 가드)."""
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from dashboard_bridge import build_status, parse_command  # noqa: E402


def test_parse_start_command():
    assert parse_command('{"command":"start","course":"A코스"}') == {
        "command": "start", "course": "A코스"}


def test_parse_estop_command():
    assert parse_command('{"command":"estop"}') == {"command": "estop", "course": None}


def test_parse_broken_json():
    assert parse_command("not json") == {"command": None, "course": None}
    assert parse_command("") == {"command": None, "course": None}


def test_build_status_roundtrip():
    assert json.loads(build_status("patrolling", "B코스")) == {
        "status": "patrolling", "course": "B코스"}


def test_build_status_idle_no_course():
    assert json.loads(build_status("idle")) == {"status": "idle", "course": None}
