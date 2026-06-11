# Copyright (c) 2024, Smart Farm Patrol Project - 꼬마 로봇 두리
# SPDX-License-Identifier: Apache-2.0
"""팔 자세 정의/전환/한계 헬퍼 단위 테스트 (torch/Isaac 불요)."""
import importlib
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import arm_poses  # noqa: E402
from arm_poses import (  # noqa: E402
    ARM_JOINTS, arm_indices, clamp_to_limits, inspect_udder_pose,
    lerp_pose, pose_vector, transition_progress,
)


# ── 자세 정의 ──────────────────────────────────────────────────────────
def test_inspect_pose_has_all_six_joints():
    p = inspect_udder_pose()
    assert set(p) == set(ARM_JOINTS)


def test_inspect_pose_env_override(monkeypatch):
    monkeypatch.setenv("SF_INSP_WR0", "-1.5")
    importlib.reload(arm_poses)
    assert arm_poses.inspect_udder_pose()["arm0_wr0"] == pytest.approx(-1.5)
    monkeypatch.delenv("SF_INSP_WR0")
    importlib.reload(arm_poses)


def test_inspect_pose_default_camera_tilts_up():
    # 손목 wr0 < 0 (카메라 상향 틸트), 어깨 sh1 < 0 (팔 낮춤) — 검사 자세 의도
    p = inspect_udder_pose()
    assert p["arm0_wr0"] < 0
    assert p["arm0_sh1"] < 0


# ── pose_vector / arm_indices ─────────────────────────────────────────
def test_pose_vector_orders_by_joint_names():
    v = pose_vector({"arm0_sh1": 1.0, "arm0_el0": 2.0})
    assert v[ARM_JOINTS.index("arm0_sh1")] == 1.0
    assert v[ARM_JOINTS.index("arm0_el0")] == 2.0
    assert v[ARM_JOINTS.index("arm0_sh0")] == 0.0      # 없는 관절 → 0


def test_arm_indices_finds_arm_joints_in_articulation():
    allj = ["fl_hx", "arm0_sh0", "fr_hy", "arm0_el0", "arm0_wr0"]
    assert arm_indices(allj) == [1, 3, 4]


# ── 한계 클램프 ────────────────────────────────────────────────────────
def test_clamp_within_and_outside():
    out = clamp_to_limits([0.0, -5.0, 5.0], [(-1, 1), (-1, 1), (-1, 1)])
    assert out == [0.0, -1.0, 1.0]


# ── 전환 보간 ──────────────────────────────────────────────────────────
def test_lerp_endpoints_and_mid():
    s, g = [0.0, 0.0], [2.0, -4.0]
    assert lerp_pose(s, g, 0.0) == [0.0, 0.0]
    assert lerp_pose(s, g, 1.0) == [2.0, -4.0]
    assert lerp_pose(s, g, 0.5) == [1.0, -2.0]


def test_lerp_clamps_t():
    s, g = [0.0], [10.0]
    assert lerp_pose(s, g, -1.0) == [0.0]      # t<0 → start
    assert lerp_pose(s, g, 2.0) == [10.0]      # t>1 → target


def test_transition_progress():
    assert transition_progress(100, 100, 60) == pytest.approx(0.0)   # 시작
    assert transition_progress(130, 100, 60) == pytest.approx(0.5)   # 중간
    assert transition_progress(200, 100, 60) == pytest.approx(1.0)   # 완료(클램프)
    assert transition_progress(50, 100, 60) == pytest.approx(0.0)    # 시작 전
    assert transition_progress(5, 0, 0) == pytest.approx(1.0)        # duration 0 → 즉시
