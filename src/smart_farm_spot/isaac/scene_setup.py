#!/usr/bin/env python3
# Copyright (c) 2024, Smart Farm Patrol Project - 꼬마 로봇 두리
# SPDX-License-Identifier: Apache-2.0
"""
scene_setup.py
==============
시나리오대로 "배치"하는 코드.

스마트 축사 자율순찰 시나리오에 맞춰:
  1. 축사 환경(environment_final.usd) 로드
  2. Spot 로봇을 순찰 시작 위치에 배치 (위치 + 방향)

⚠️ 소(cow) 스폰/생성은 여기서 하지 않습니다. (팀에서 작성 중인 "생성 코드")
   spawn_cows() 에 연결 지점(hook)만 비워 두었습니다.

[두 가지 사용 방식]
  A) 단독 실행 — 배치 결과를 GUI로 바로 확인
       source ~/dev_ws/venv/isaaclab/bin/activate
       cd ~/dev_ws/isaac_sim/IsaacLab
       ./isaaclab.sh -p ~/dev_ws/isaac_sim/smart_farm_spot/isaac/scene_setup.py

  B) 모듈 import — 통합 브릿지/뷰어가 배치 로직 재사용
       import scene_setup
       scene_setup.setup_scenario()              # 환경 로드 + 로봇 배치
       scene_setup.place_robot_pose(articulation)  # 정밀 포즈 (reset/initialize 후)

[배치 파라미터] — 아래 상수를 축사 좌표에 맞게 조정
"""

import math
import os

# ══════════════════════════════════════════════════════════════════════
# 시나리오 배치 상수 (축사 좌표/USD 경로에 맞춰 조정)
# ══════════════════════════════════════════════════════════════════════
# 워크스페이스 내장 에셋 (포터블: 패키지 상대경로)
_ASSETS    = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "assets")
ENV_USD    = os.path.join(_ASSETS, "scene", "environment_final.usd")
ROBOT_USD  = os.path.join(_ASSETS, "robot", "spot_with_arm.usd")
ROBOT_PRIM = "/World/Spot"

# 순찰 시작 위치 — waypoints.yaml 의 "대기 위치(0,0)" / 통로 진입(+x) 기준
START_POS      = (0.0, 0.0, 0.65)   # (x, y, z) m  — z 는 낙하/뒤집힘 방지 높이
START_YAW_DEG  = 0.0                # +x 방향(통로 진입) 바라보게

# upAxis 보정 쿼터니언 (wxyz). Z-up 정상이면 단위(1,0,0,0).
# spot USD 가 뒤집혀 보이면 PROGRESS_PROCEDURE.md 의 A-2 후보로 교체:
#   Y-up → (0.7071, -0.7071, 0.0, 0.0)
BASE_ROT_WXYZ  = (1.0, 0.0, 0.0, 0.0)


# ══════════════════════════════════════════════════════════════════════
# 쿼터니언 유틸
# ══════════════════════════════════════════════════════════════════════
def yaw_to_quat(yaw_deg):
    """Z축 yaw(도) → (w, x, y, z)."""
    h = math.radians(yaw_deg) / 2.0
    return (math.cos(h), 0.0, 0.0, math.sin(h))


def quat_mul(a, b):
    """쿼터니언 곱 a*b, 둘 다 (w, x, y, z)."""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def start_quat():
    """upAxis 보정 + 시작 yaw 를 합친 최종 배치 방향."""
    return quat_mul(BASE_ROT_WXYZ, yaw_to_quat(START_YAW_DEG))


# ══════════════════════════════════════════════════════════════════════
# 배치 (lazy import — SimulationApp 생성 후 호출되어야 함)
# ══════════════════════════════════════════════════════════════════════
def load_environment(env_usd=ENV_USD):
    """축사 환경 USD 를 현재 스테이지로 로드."""
    import omni.usd
    if not os.path.exists(env_usd):
        import carb
        carb.log_error(f"환경 USD 없음: {env_usd}")
        return False
    print(f"[배치] 환경 로드: {env_usd}")
    omni.usd.get_context().open_stage(env_usd)
    return True


def add_robot(robot_usd=ROBOT_USD, prim_path=ROBOT_PRIM, pos=START_POS):
    """로봇 USD 를 씬에 추가하고 시작 위치로 1차 배치(translate)."""
    from isaacsim.core.utils.stage import add_reference_to_stage
    import isaacsim.core.utils.prims as prim_utils
    if not os.path.exists(robot_usd):
        import carb
        carb.log_error(f"로봇 USD 없음: {robot_usd}")
        return False
    print(f"[배치] 로봇 추가: {robot_usd}  →  {prim_path}  @ {pos}")
    add_reference_to_stage(usd_path=robot_usd, prim_path=prim_path)
    prim_utils.set_prim_attribute_value(prim_path, "xformOp:translate", list(pos))
    return True


def place_robot_pose(articulation, pos=START_POS, quat=None):
    """
    물리 초기화(world.reset + articulation.initialize) 후 정밀 포즈 설정.
    위치 + 방향(upAxis 보정 + yaw) 을 한 번에 적용.
    """
    import numpy as np
    quat = quat if quat is not None else start_quat()
    articulation.set_world_pose(position=np.array(pos, dtype=float),
                                orientation=np.array(quat, dtype=float))
    print(f"[배치] 로봇 포즈 확정: pos={pos}, quat(wxyz)={tuple(round(q, 4) for q in quat)}")


def spawn_cows(stage=None):
    """
    ┌──────────────────────────────────────────────────────────────┐
    │  ★ 소(cow) 스폰/생성 코드 연결 지점 — 팀에서 작성 중 ★         │
    │                                                                │
    │  여기에서 애니메이션 소(cow_idle/eat/walk/LimpWalk)를          │
    │  축사 각 칸(stall)에 배치/생성하면 됩니다.                      │
    │  배치 후 setup_semantics.tag_semantics()로 'cow'/'udder'/'leg' │
    │  라벨을 부여하면 /spot/arm/detections 로 박스가 나갑니다.       │
    └──────────────────────────────────────────────────────────────┘
    """
    pass  # 생성 코드 미연결 (의도적으로 비움)


def setup_scenario(env_usd=ENV_USD, robot_usd=ROBOT_USD,
                   prim_path=ROBOT_PRIM, pos=START_POS):
    """
    시나리오 배치 일괄 실행: 환경 로드 → 로봇 추가.
    (정밀 포즈는 reset/initialize 후 place_robot_pose 로)
    통합 브릿지/뷰어가 import 해서 재사용.
    """
    load_environment(env_usd)
    add_robot(robot_usd, prim_path, pos)
    return prim_path


# ══════════════════════════════════════════════════════════════════════
# 단독 실행 — 배치 결과를 GUI 로 확인
# ══════════════════════════════════════════════════════════════════════
def main():
    import argparse
    parser = argparse.ArgumentParser(description="시나리오 배치 (환경+로봇)")
    parser.add_argument("--env-usd", type=str, default=ENV_USD)
    parser.add_argument("--robot-usd", type=str, default=ROBOT_USD)
    parser.add_argument("--robot-prim", type=str, default=ROBOT_PRIM)
    parser.add_argument("--start-x", type=float, default=START_POS[0])
    parser.add_argument("--start-y", type=float, default=START_POS[1])
    parser.add_argument("--start-z", type=float, default=START_POS[2])
    parser.add_argument("--start-yaw", type=float, default=START_YAW_DEG)
    parser.add_argument("--headless", action="store_true", help="GUI 없이 실행")
    a = parser.parse_args()

    # IsaacLab AppLauncher 로 앱 생성 (다른 isaacsim import보다 반드시 먼저).
    # ※ 이 환경에선 bare `SimulationApp` 이 omni.kit.usd 에서 깨지므로 AppLauncher 사용.
    from isaaclab.app import AppLauncher
    app = AppLauncher(headless=a.headless).app

    from isaacsim.core.api import World
    from isaacsim.core.prims import SingleArticulation

    pos = (a.start_x, a.start_y, a.start_z)
    quat = quat_mul(BASE_ROT_WXYZ, yaw_to_quat(a.start_yaw))

    print("=" * 60)
    print(" 시나리오 배치 (환경 + 로봇)")
    print("=" * 60)

    world = World(stage_units_in_meters=1.0)

    # ── 배치 ──────────────────────────────────────────────────────
    setup_scenario(a.env_usd, a.robot_usd, a.robot_prim, pos)
    spawn_cows()   # 미연결 (생성 코드 작성 중)

    # ── 물리 초기화 + 정밀 포즈 ───────────────────────────────────
    robot = SingleArticulation(prim_path=a.robot_prim, name="spot")
    world.reset()
    robot.initialize()
    place_robot_pose(robot, pos=pos, quat=quat)
    app.update()

    print("\n[보기] GUI 에서 배치 확인. Play(▶) 로 물리/애니메이션 시작.")
    print("       (창 닫기 또는 Ctrl+C 로 종료)\n")

    try:
        while app.is_running():
            world.step(render=True)
    except KeyboardInterrupt:
        print("\n[종료]")
    finally:
        app.close()


if __name__ == "__main__":
    main()
