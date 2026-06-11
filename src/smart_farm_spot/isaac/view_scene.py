#!/usr/bin/env python3
# Copyright (c) 2024, Smart Farm Patrol Project - 꼬마 로봇 두리
# SPDX-License-Identifier: Apache-2.0
"""
view_scene.py
=============
"일단 로봇/환경부터 눈으로 보기" 용 최소 GUI 뷰어.

ROS 2 · 센서 브릿지 · Nav 제어 전부 빼고, USD만 GUI로 띄워서 확인합니다.
(통합 브릿지 isaac_sim_bridge.py 의 무거운 부분 없이 가볍게 시작)

[이 PC 실행법] — 이 환경엔 ~/isaacsim/python.sh 가 없고 Isaac Sim 5.1 이
venv 에 설치돼 있으므로 아래처럼 실행:

  source ~/dev_ws/venv/isaaclab/bin/activate
  cd ~/dev_ws/isaac_sim/IsaacLab
  ./isaaclab.sh -p ~/dev_ws/isaac_sim/smart_farm_spot/isaac/view_scene.py [옵션]

[옵션]
  --usd PATH        열어볼 씬 USD (예: 축사 environment_final.usd)
  --robot-usd PATH  씬 위에 추가로 얹을 로봇 USD (예: spot_with_arm.usd)
  --robot-prim STR  로봇을 놓을 prim 경로 (기본 /World/Spot)
  --spawn-z FLOAT   로봇 스폰 높이 (기본 0.65, 뒤집힘/낙하 방지)

[예시]
  # 1) 축사+소 환경만 보기
  ... view_scene.py --usd ~/dev_ws/isaac_sim/environment_final.usd

  # 2) 로봇만 보기 (기본 지면 위)
  ... view_scene.py --robot-usd ~/dev_ws/isaac_sim/spot_with_arm.usd

  # 3) 축사 위에 로봇까지 얹어 보기
  ... view_scene.py --usd ~/dev_ws/isaac_sim/environment_final.usd \
                    --robot-usd ~/dev_ws/isaac_sim/spot_with_arm.usd
"""

import argparse
import os

# ── 인자 파싱 → SimulationApp 먼저 (다른 import보다 반드시 먼저) ──────
parser = argparse.ArgumentParser(description="Spot/축사 USD 최소 GUI 뷰어")
parser.add_argument("--usd", type=str, default=None, help="열어볼 씬 USD 경로")
parser.add_argument("--robot-usd", type=str, default=None, help="추가로 얹을 로봇 USD")
parser.add_argument("--robot-prim", type=str, default="/World/Spot", help="로봇 prim 경로")
parser.add_argument("--spawn-z", type=float, default=0.65, help="로봇 스폰 높이(m)")
args = parser.parse_args()

# IsaacLab AppLauncher 로 앱 생성 (bare SimulationApp 은 이 환경에서 omni.kit.usd 에러).
from isaaclab.app import AppLauncher  # noqa: E402
# 항상 GUI (보는 게 목적)
simulation_app = AppLauncher(headless=False).app

import omni.usd  # noqa: E402
import carb  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.utils.stage import add_reference_to_stage  # noqa: E402
import isaacsim.core.utils.prims as prim_utils  # noqa: E402


def main():
    print("=" * 60)
    print(" Spot/축사 USD 뷰어 (GUI)")
    print("=" * 60)

    world = World(stage_units_in_meters=1.0)

    # ── 씬 로드 ───────────────────────────────────────────────────
    if args.usd and os.path.exists(args.usd):
        print(f"[로드] 씬 USD: {args.usd}")
        omni.usd.get_context().open_stage(args.usd)
        simulation_app.update()
    else:
        if args.usd:
            carb.log_warn(f"씬 USD 없음(무시): {args.usd}")
        print("[로드] 기본 지면 생성")
        world.scene.add_default_ground_plane()
        simulation_app.update()

    # ── 로봇 추가 (선택) ──────────────────────────────────────────
    if args.robot_usd:
        if os.path.exists(args.robot_usd):
            print(f"[로드] 로봇 USD: {args.robot_usd}  →  {args.robot_prim}")
            add_reference_to_stage(usd_path=args.robot_usd, prim_path=args.robot_prim)
            prim_utils.set_prim_attribute_value(
                args.robot_prim, "xformOp:translate", [0.0, 0.0, args.spawn_z])
            simulation_app.update()
        else:
            carb.log_warn(f"로봇 USD 없음(무시): {args.robot_usd}")

    world.reset()
    simulation_app.update()

    print("\n[보기] GUI 창에서 마우스로 둘러보세요. (창 닫기 또는 Ctrl+C 로 종료)")
    print("       Play(▶) 를 누르면 물리 시뮬이 시작됩니다.\n")

    try:
        while simulation_app.is_running():
            world.step(render=True)
    except KeyboardInterrupt:
        print("\n[종료]")
    finally:
        simulation_app.close()


if __name__ == "__main__":
    main()
