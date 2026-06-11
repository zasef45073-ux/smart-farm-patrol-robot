#!/usr/bin/env python3
# Copyright (c) 2024, Smart Farm Patrol Project - 꼬마 로봇 두리
# SPDX-License-Identifier: Apache-2.0
"""
capture_check.py
================
중간 점검용 — 로봇이 실제로 "생성"되는지 헤드리스로 확인하고 스크린샷 저장.

GUI 없이:
  1. 시나리오 배치(scene_setup.setup_scenario) 실행
  2. 물리 초기화 후 articulation 정보 출력 (DOF/관절/베이스 포즈) → 생성 증거
  3. 카메라로 한 장 캡처 → PNG 저장 (best-effort)

실행:
  source ~/dev_ws/venv/isaaclab/bin/activate
  cd ~/dev_ws/isaac_sim/IsaacLab
  ./isaaclab.sh -p ~/dev_ws/isaac_sim/smart_farm_spot/isaac/capture_check.py
"""

import os
import sys

OUT_PNG = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "_output", "_check_robot.png")
os.makedirs(os.path.dirname(OUT_PNG), exist_ok=True)
SETTLE_FRAMES = 60

# IsaacLab AppLauncher 로 앱 생성 (bare SimulationApp 은 이 환경에서 omni.kit.usd 에러).
from isaaclab.app import AppLauncher  # noqa: E402
app = AppLauncher(headless=True).app

import numpy as np  # noqa: E402
import carb  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.prims import SingleArticulation  # noqa: E402

# scene_setup 같은 폴더에서 import
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import scene_setup  # noqa: E402


def main():
    print("=" * 60)
    print(" 중간 점검: 로봇 생성 확인 (헤드리스)")
    print("=" * 60)

    world = World(stage_units_in_meters=1.0)

    # ── 배치 ──────────────────────────────────────────────────────
    scene_setup.setup_scenario()
    app.update()

    # ── 물리 초기화 + 정밀 포즈 ───────────────────────────────────
    robot = SingleArticulation(prim_path=scene_setup.ROBOT_PRIM, name="spot")
    world.reset()
    robot.initialize()
    scene_setup.place_robot_pose(robot)

    # ── 안정화 스텝 ───────────────────────────────────────────────
    for _ in range(SETTLE_FRAMES):
        world.step(render=True)

    # ── 생성 증거 출력 ────────────────────────────────────────────
    print("\n[생성 점검]")
    try:
        dof = robot.num_dof
        names = robot.dof_names
        pos, quat = robot.get_world_pose()
        print(f"  ✅ articulation 초기화됨")
        print(f"  ✅ DOF 개수    : {dof}")
        print(f"  ✅ 관절 이름   : {list(names)}")
        print(f"  ✅ 베이스 위치 : ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})")
        print(f"  ✅ 베이스 방향 : wxyz {tuple(round(float(q),4) for q in quat)}")
    except Exception as e:
        print(f"  ⚠️  articulation 정보 조회 실패: {e}")

    # ── 스크린샷 캡처 (best-effort) ───────────────────────────────
    captured = _capture(OUT_PNG)
    if captured:
        print(f"\n  📸 스크린샷 저장: {OUT_PNG}")
    else:
        print("\n  ⚠️  스크린샷 캡처 실패 (위 텍스트 점검으로 생성은 확인됨)")

    app.close()


def _capture(path: str) -> bool:
    """replicator 카메라로 로봇을 비스듬히 내려다보며 한 장 캡처 → PNG."""
    try:
        import omni.replicator.core as rep

        rx, ry, rz = scene_setup.START_POS
        cam = rep.create.camera(
            position=(rx + 3.0, ry - 3.0, rz + 2.5),
            look_at=(rx, ry, rz),
        )
        rp = rep.create.render_product(cam, (1280, 720))
        rgb = rep.AnnotatorRegistry.get_annotator("rgb")
        rgb.attach([rp])

        # 렌더 파이프라인 몇 스텝 돌려 이미지 확보
        for _ in range(10):
            rep.orchestrator.step(rt_subframes=4)

        data = rgb.get_data()  # (H, W, 4) uint8
        if data is None or getattr(data, "size", 0) == 0:
            return False
        arr = np.asarray(data)[:, :, :3]
        return _save_png(arr, path)
    except Exception as e:
        carb.log_warn(f"캡처 예외: {e}")
        return False


def _save_png(arr: "np.ndarray", path: str) -> bool:
    """numpy RGB 배열을 PNG 로 저장 (PIL → imageio 폴백)."""
    try:
        from PIL import Image
        Image.fromarray(arr.astype(np.uint8), "RGB").save(path)
        return True
    except Exception:
        pass
    try:
        import imageio.v2 as imageio
        imageio.imwrite(path, arr.astype(np.uint8))
        return True
    except Exception as e:
        carb.log_warn(f"PNG 저장 실패: {e}")
        return False


if __name__ == "__main__":
    main()
