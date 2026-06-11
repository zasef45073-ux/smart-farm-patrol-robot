#!/usr/bin/env python3
# Copyright (c) 2024, Smart Farm Patrol Project - 꼬마 로봇 두리
# SPDX-License-Identifier: Apache-2.0
"""
make_keepout_mask.py
====================
**투명(센서 미감지) 가상벽 → Nav2 Keepout 마스크 생성**.

barn_map_server 가 래스터화한 점유격자(투명 가상벽 포함)에서 keepout 마스크
(map_server 가 읽는 pgm+yaml)를 만든다. KeepoutFilter 가 이 마스크를 읽어
**라이다가 못 보는 가상벽도 코스트맵에서 회피**한다(SLAM 모드 포함).

입력: /tmp/barn_map.npy (H×W int8: -1/0/100) + /tmp/barn_map.json (res/origin)
출력: maps/keepout_mask.pgm (+ .yaml)

실행: python3 tools/make_keepout_mask.py
"""
import json
import os

import numpy as np

_PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def to_keepout_pixels(grid):
    """점유격자(int8) → keepout pgm 픽셀(uint8).

    점유(100)=keepout=검정(0), 그 외(자유/미지)=흰색(254).
    pgm 은 상단 행부터 저장 → occupancy(원점 좌하)와 맞추려 상하 반전(flipud).
    """
    g = np.asarray(grid)
    px = np.where(g == 100, 0, 254).astype(np.uint8)
    return np.flipud(px)


def write_pgm(path, px):
    """P5(이진) pgm 저장."""
    h, w = px.shape
    with open(path, "wb") as f:
        f.write(f"P5\n{w} {h}\n255\n".encode("ascii"))
        f.write(px.tobytes())


def write_yaml(path, pgm_name, resolution, origin):
    with open(path, "w") as f:
        f.write(f"image: {pgm_name}\n")
        f.write(f"resolution: {resolution}\n")
        f.write(f"origin: [{origin[0]}, {origin[1]}, 0.0]\n")
        f.write("negate: 0\n")
        f.write("occupied_thresh: 0.65\n")
        f.write("free_thresh: 0.196\n")
        f.write("mode: trinary\n")   # KeepoutFilter: 점유=keepout


def main():
    npy = os.environ.get("SF_BARN_MAP_NPY", "/tmp/barn_map.npy")
    js = os.environ.get("SF_BARN_MAP_JSON", "/tmp/barn_map.json")
    grid = np.load(npy)
    with open(js) as f:
        meta = json.load(f)
    px = to_keepout_pixels(grid)
    out_dir = os.path.join(_PKG, "maps")
    os.makedirs(out_dir, exist_ok=True)
    write_pgm(os.path.join(out_dir, "keepout_mask.pgm"), px)
    write_yaml(os.path.join(out_dir, "keepout_mask.yaml"), "keepout_mask.pgm",
               meta["resolution"], meta["origin"])
    n_keep = int((grid == 100).sum())
    print(f"✅ keepout 마스크 생성: {px.shape[1]}×{px.shape[0]} "
          f"keepout셀 {n_keep} → maps/keepout_mask.pgm/.yaml")


if __name__ == "__main__":
    main()
