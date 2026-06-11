# Copyright (c) 2024, Smart Farm Patrol Project - 꼬마 로봇 두리
# SPDX-License-Identifier: Apache-2.0
"""YOLO 소 트랙 상태관리 단위 테스트 (ROS/YOLO/학습 불요)."""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cow_tracker import CowTrackManager  # noqa: E402


def det(tid, conf=0.9, cx=0.0, cy=0.0):
    return {"id": tid, "conf": conf, "cx": cx, "cy": cy}


# ── 확정(플리커 흡수) ──────────────────────────────────────────────────
def test_not_confirmed_before_min_hits():
    m = CowTrackManager(min_hits=3)
    assert m.update([det(1)], now=0.0) is None      # 1 hit
    assert m.update([det(1)], now=0.1) is None      # 2 hits
    assert m.update([det(1)], now=0.2) == 1         # 3 hits → 확정·대상
    assert 1 in m.confirmed()


def test_single_frame_flicker_ignored():
    m = CowTrackManager(min_hits=3)
    m.update([det(7)], now=0.0)                      # 깜빡 1프레임
    assert m.update([], now=0.05) is None           # 사라짐 → 대상 없음
    assert m.confirmed() == []


# ── 드롭아웃(노화) ─────────────────────────────────────────────────────
def test_track_ages_out_after_max_age():
    m = CowTrackManager(min_hits=1, max_age=0.5)
    assert m.update([det(2)], now=0.0) == 2
    # max_age 초과 미검출 → 소멸
    assert m.update([], now=0.6) is None
    assert 2 not in m.confirmed()


def test_brief_gap_within_max_age_kept():
    m = CowTrackManager(min_hits=1, max_age=0.5)
    m.update([det(2)], now=0.0)
    assert m.update([], now=0.3) == 2               # 0.3s 공백은 유지
    assert m.update([det(2)], now=0.4) == 2


# ── 대상 락(검사 끝날 때까지 안 튐) ───────────────────────────────────
def test_target_locked_until_inspected():
    m = CowTrackManager(min_hits=1)
    # 소1(conf 0.9) 대상 확정
    assert m.update([det(1, conf=0.9)], now=0.0) == 1
    # 더 높은 conf 소2 등장해도 락이라 대상 유지
    assert m.update([det(1, 0.9), det(2, 0.99)], now=0.1) == 1
    # 소1 검사 완료 → 이제 소2로 전환
    m.mark_inspected()
    assert m.update([det(1, 0.9), det(2, 0.99)], now=0.2) == 2


def test_no_lock_picks_highest_conf():
    m = CowTrackManager(min_hits=1, lock_until_inspected=False)
    m.update([det(1, conf=0.8)], now=0.0)
    assert m.update([det(1, 0.8), det(2, 0.95)], now=0.1) == 2   # 락 없으면 최고 conf


# ── 검사 완료 / 완주 ──────────────────────────────────────────────────
def test_inspected_not_reselected():
    m = CowTrackManager(min_hits=1)
    m.update([det(1)], now=0.0)
    m.mark_inspected(1)
    # 소1만 계속 보여도 이미 검사됨 → 대상 없음
    assert m.update([det(1)], now=0.1) is None


def test_all_inspected_completion():
    m = CowTrackManager(min_hits=1)
    m.update([det(1), det(2)], now=0.0)
    assert not m.all_inspected()
    m.mark_inspected(1)
    assert not m.all_inspected()
    m.mark_inspected(2)
    assert m.all_inspected()                         # 본 소 전부 검사 → 완주


def test_all_inspected_with_expected_set():
    m = CowTrackManager(min_hits=1)
    m.update([det(1)], now=0.0)
    m.mark_inspected(1)
    assert not m.all_inspected(expected_ids=[1, 2, 3])   # 아직 2,3 남음
    assert m.all_inspected(expected_ids=[1])


def test_empty_state_not_done():
    m = CowTrackManager()
    assert m.all_inspected() is False                # 본 소 없음 → 완주 아님


# ── target_obs 접근 ───────────────────────────────────────────────────
def test_target_obs_returns_last_observation():
    m = CowTrackManager(min_hits=1)
    m.update([det(5, conf=0.7, cx=100, cy=50)], now=0.0)
    obs = m.target_obs()
    assert obs["id"] == 5 and obs["cx"] == 100 and obs["cy"] == 50
    m.mark_inspected(5)
    assert m.target_obs() is None
