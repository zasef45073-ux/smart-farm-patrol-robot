# Copyright (c) 2024, Smart Farm Patrol Project - 꼬마 로봇 두리
# SPDX-License-Identifier: Apache-2.0
"""keepout 마스크 변환 로직 테스트 (numpy)."""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "tools"))

from make_keepout_mask import to_keepout_pixels  # noqa: E402


def test_occupied_becomes_black_free_white():
    grid = np.array([[100, 0], [-1, 100]], dtype=np.int8)
    px = to_keepout_pixels(grid)
    # flipud 후: 아래행이 위로 → 원래 [-1,100] 행이 top
    assert px.shape == (2, 2)
    assert px.dtype == np.uint8
    # 점유(100)=0, 자유/미지=254
    assert set(np.unique(px).tolist()) <= {0, 254}
    assert (px == 0).sum() == 2      # 100 셀 2개
    assert (px == 254).sum() == 2    # 0,-1 셀


def test_flip_vertical_orientation():
    # 위쪽 행만 점유 → flipud 후 아래쪽 행이 점유여야(원점 좌하 보정)
    grid = np.array([[100, 100], [0, 0]], dtype=np.int8)
    px = to_keepout_pixels(grid)
    assert (px[0] == 254).all()      # top row 자유
    assert (px[1] == 0).all()        # bottom row keepout


def test_all_free_no_keepout():
    grid = np.zeros((3, 3), dtype=np.int8)
    px = to_keepout_pixels(grid)
    assert (px == 254).all()
