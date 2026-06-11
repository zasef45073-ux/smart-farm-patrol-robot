# Copyright (c) 2024, Smart Farm Patrol Project - 꼬마 로봇 두리
# SPDX-License-Identifier: Apache-2.0
"""팔 경량화 순수 헬퍼 테스트 (Isaac/torch 불요)."""
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from arm_mass import arm_body_indices, inertia_ratio, lighten_masses  # noqa: E402


def test_arm_body_indices_by_prefix():
    names = ["body", "fl_hx", "arm0_link_sh0", "arm0_link_wr1", "imu"]
    assert arm_body_indices(names) == [2, 3]


def test_arm_body_indices_no_arm():
    assert arm_body_indices(["body", "fl_hx", "fr_kn"]) == []


def test_lighten_masses_only_arm():
    masses = [10.0, 2.0, 3.0, 4.0]
    out = lighten_masses(masses, [1, 3], light_kg=0.01)
    assert out == [10.0, 0.01, 3.0, 0.01]
    assert masses == [10.0, 2.0, 3.0, 4.0]   # 원본 불변


def test_inertia_ratio_proportional():
    assert inertia_ratio(2.0, 0.02) == pytest.approx(0.01)   # 0.02/2.0
    assert inertia_ratio(0.0, 0.01) == 1.0                   # 0 보호
    assert inertia_ratio(-1.0, 0.01) == 1.0
