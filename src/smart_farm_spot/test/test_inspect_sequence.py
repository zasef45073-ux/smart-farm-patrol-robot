# Copyright (c) 2024, Smart Farm Patrol Project - 꼬마 로봇 두리
# SPDX-License-Identifier: Apache-2.0
"""후방 도착→앉기→검사→복귀 상태머신 테스트 (Isaac/torch 불요)."""
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from inspect_sequence import InspectSequence  # noqa: E402


def seq():
    return InspectSequence(sit_steps=10, deploy_steps=10, hold_steps=10,
                           retract_steps=10, rise_steps=10)


def test_idle_before_start():
    s = seq().update(0)
    assert s.phase == "IDLE" and not s.active and not s.cmd_zero


def test_sit_phase_lowers_legs():
    q = seq(); q.start(0)
    s = q.update(5)                      # SIT 중간
    assert s.phase == "SIT"
    assert s.active and s.cmd_zero       # 검사 중 주행 금지
    assert s.leg_t == pytest.approx(0.5)
    assert s.arm_t == 0.0


def test_deploy_after_sit_arm_ramps():
    q = seq(); q.start(0)
    s = q.update(15)                     # SIT(10)+5
    assert s.phase == "DEPLOY"
    assert s.leg_t == 1.0                # 완전히 앉음
    assert s.arm_t == pytest.approx(0.5)


def test_hold_captures_once():
    q = seq(); q.start(0)
    s1 = q.update(20)                    # HOLD 진입(10+10)
    assert s1.phase == "HOLD"
    assert s1.leg_t == 1.0 and s1.arm_t == 1.0
    assert s1.capture is True            # 첫 프레임 촬영
    s2 = q.update(21)
    assert s2.capture is False           # 중복 촬영 안 함


def test_retract_arm_returns():
    q = seq(); q.start(0)
    s = q.update(35)                     # RETRACT 중간(30+5)
    assert s.phase == "RETRACT"
    assert s.leg_t == 1.0
    assert s.arm_t == pytest.approx(0.5)


def test_rise_stands_up():
    q = seq(); q.start(0)
    s = q.update(45)                     # RISE 중간(40+5)
    assert s.phase == "RISE"
    assert s.leg_t == pytest.approx(0.5)  # 일어서는 중
    assert s.arm_t == 0.0


def test_done_after_full_sequence():
    q = seq(); q.start(0)
    s = q.update(60)                     # 전체(50) 초과
    assert s.phase == "DONE"
    assert s.done and not s.active and not s.cmd_zero


def test_cmd_zero_throughout_active():
    q = seq(); q.start(0)
    for step in (3, 15, 25, 35, 45):     # SIT~RISE 전 구간
        assert q.update(step).cmd_zero is True


def test_start_offset_step():
    q = seq(); q.start(1000)
    s = q.update(1005)                   # 시작 1000 기준 +5
    assert s.phase == "SIT" and s.leg_t == pytest.approx(0.5)


def test_reset_returns_to_idle():
    q = seq(); q.start(0)
    q.update(20)
    q.reset()
    assert not q.is_active()
    assert q.update(25).phase == "IDLE"
