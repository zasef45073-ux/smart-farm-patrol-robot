#!/usr/bin/env python3
# Copyright (c) 2024, Smart Farm Patrol Project - 꼬마 로봇 두리
# SPDX-License-Identifier: Apache-2.0
"""
drive_course.py
===============
H자(또는 임의) 주행코스를 로봇이 kinematic 으로 주행하는 데모.

Nav2 없이, 로봇 베이스에 cmd_vel 을 직접 적용(회전→전진)하여
웨이포인트를 순서대로 따라갑니다. 바닥에 H 코스 라인을 그려 보여줍니다.
(RL 보행 정책 없이 베이스를 미끄러뜨리는 kinematic 모드 — 빠른 시각 확인용)

실행:
  source ~/dev_ws/venv/isaaclab/bin/activate
  cd ~/dev_ws/isaac_sim/IsaacLab
  ./isaaclab.sh -p ~/dev_ws/spot_ws/src/smart_farm_spot/isaac/drive_course.py
  # 옵션: --loop (무한 반복)  --sensors (센서도 부착)  --headless
"""

import argparse
import math
import os

# ── H자 코스 웨이포인트 (x, y) — 한 붓 그리기 ────────────────────────
H_WAYPOINTS = [
    (0.0, 0.0),    # ① 좌하 시작
    (0.0, 3.0),    # ② 좌측 세로 ↑
    (0.0, 1.5),    # ③ 좌측 중간 복귀
    (2.0, 1.5),    # ④ 가로 바 →
    (2.0, 3.0),    # ⑤ 우측 세로 ↑
    (2.0, 0.0),    # ⑥ 우측 세로 ↓ (끝)
]

# H자 형태 라인 (시각화용 — 실제 H 모양 3획)
H_STROKES = [
    ((0.0, 0.0), (0.0, 3.0)),     # 왼쪽 세로
    ((2.0, 0.0), (2.0, 3.0)),     # 오른쪽 세로
    ((0.0, 1.5), (2.0, 1.5)),     # 가로 바
]

# ── 주행 파라미터 ────────────────────────────────────────────────────
V_MAX      = 0.8     # m/s 전진 최대
W_MAX      = 1.2     # rad/s 회전 최대
K_ANG      = 2.0     # 방향오차 → 각속도 게인
WP_TOL     = 0.20    # m 웨이포인트 도달 판정
HEAD_TOL   = 0.15    # rad 이내면 전진 시작
SPAWN_Z    = 0.65    # 베이스 높이(미끄러짐 유지)

# upAxis 보정 — spot_with_arm.usd 는 그냥 두면 등으로 뒤집혀 소환됨.
# X축 180° 뒤집기로 직립.
BASE_ROT_WXYZ = (0.0, 1.0, 0.0, 0.0)

# 로봇 로컬 전방축 보정(도) — X180 뒤집기로 전방이 뒤집혀서 180 으로 정렬.
FACE_YAW_OFFSET = 180.0


def _quat_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw*bw - ax*bx - ay*by - az*bz,
            aw*bx + ax*bw + ay*bz - az*by,
            aw*by - ax*bz + ay*bw + az*bx,
            aw*bz + ax*by - ay*bx + az*bw)


def _face_quat(heading):
    """진행방향(heading, rad)을 바라보는 직립 자세 쿼터니언."""
    h = (heading + math.radians(FACE_YAW_OFFSET)) / 2.0
    return _quat_mul((math.cos(h), 0.0, 0.0, math.sin(h)), BASE_ROT_WXYZ)


def main():
    parser = argparse.ArgumentParser(description="H자 코스 kinematic 주행")
    _assets = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "assets")
    parser.add_argument("--robot-usd", type=str,
                        default=os.path.join(_assets, "robot", "spot_with_arm.usd"))
    parser.add_argument("--robot-prim", type=str, default="/World/Spot")
    parser.add_argument("--loop", action="store_true", help="코스 무한 반복")
    parser.add_argument("--sensors", action="store_true", help="센서도 부착")
    parser.add_argument("--headless", action="store_true")
    a = parser.parse_args()

    # IsaacLab AppLauncher (bare SimulationApp 은 이 환경에서 omni.kit.usd 에러)
    from isaaclab.app import AppLauncher
    app = AppLauncher(headless=a.headless).app

    import numpy as np
    from isaacsim.core.api import World
    from isaacsim.core.utils.stage import add_reference_to_stage
    from isaacsim.core.prims import SingleArticulation
    import isaacsim.core.utils.prims as prim_utils

    print("=" * 60)
    print(" H자 코스 kinematic 주행 데모")
    print("=" * 60)

    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()

    if not os.path.exists(a.robot_usd):
        print(f"❌ 로봇 USD 없음: {a.robot_usd}")
        app.close()
        return
    add_reference_to_stage(usd_path=a.robot_usd, prim_path=a.robot_prim)
    # 시작 위치 = 첫 웨이포인트
    sx, sy = H_WAYPOINTS[0]
    prim_utils.set_prim_attribute_value(
        a.robot_prim, "xformOp:translate", [sx, sy, SPAWN_Z])
    app.update()

    # 센서 부착 (선택)
    if a.sensors:
        import sys
        sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
        try:
            import add_sensors
            add_sensors.add_sensors(a.robot_prim)
        except Exception as e:
            print(f"  ⚠️ 센서 부착 건너뜀: {e}")

    robot = SingleArticulation(prim_path=a.robot_prim, name="spot")
    world.reset()
    robot.initialize()
    # 직립 소환
    robot.set_world_pose(position=np.array([sx, sy, SPAWN_Z]),
                         orientation=np.array(BASE_ROT_WXYZ))
    app.update()

    # 카메라를 코스 위로
    try:
        from isaacsim.core.utils.viewports import set_camera_view
        set_camera_view(eye=[1.0, -4.5, 4.5], target=[1.0, 1.5, 0.0])
    except Exception:
        pass

    # 디버그 드로우 핸들
    try:
        from isaacsim.util.debug_draw import _debug_draw
        draw = _debug_draw.acquire_debug_draw_interface()
    except Exception:
        draw = None

    print(f"\n[주행] 웨이포인트 {len(H_WAYPOINTS)}개 H자 코스 시작"
          f"{' (무한 반복)' if a.loop else ''}\n")

    idx = 1                       # 0번은 시작 위치, 1번부터 목표
    done = False
    try:
        while app.is_running():
            world.step(render=True)

            # ── H자 라인 + 웨이포인트 시각화 (매 프레임) ──
            if draw is not None:
                _draw_course(draw, idx)

            if done:
                continue

            pos, _ = robot.get_world_pose()
            x, y = float(pos[0]), float(pos[1])
            tx, ty = H_WAYPOINTS[idx]
            dx, dy = tx - x, ty - y
            dist = math.hypot(dx, dy)

            if dist < WP_TOL:
                idx += 1
                if idx >= len(H_WAYPOINTS):
                    if a.loop:
                        idx = 1
                        robot.set_world_pose(
                            position=np.array([H_WAYPOINTS[0][0],
                                               H_WAYPOINTS[0][1], SPAWN_Z]),
                            orientation=np.array(BASE_ROT_WXYZ))
                    else:
                        print("  ✅ H자 코스 완주!")
                        robot.set_linear_velocity(np.zeros(3))
                        robot.set_angular_velocity(np.zeros(3))
                        done = True
                    continue
                print(f"  → 웨이포인트 {idx+1}/{len(H_WAYPOINTS)} 이동 "
                      f"({H_WAYPOINTS[idx][0]}, {H_WAYPOINTS[idx][1]})")
                continue

            # ── 진행방향 바라보며 직진 (z 고정으로 가라앉음 방지) ──
            heading = math.atan2(dy, dx)
            # z 핀 + 진행방향 정렬 (xy 는 현재값 유지 → 텔레포트 아님)
            robot.set_world_pose(position=np.array([x, y, SPAWN_Z]),
                                 orientation=np.array(_face_quat(heading)))
            ux, uy = dx / dist, dy / dist
            v = min(V_MAX, dist * 2.0)
            robot.set_linear_velocity(np.array([ux * v, uy * v, 0.0]))
            robot.set_angular_velocity(np.zeros(3))

    except KeyboardInterrupt:
        print("\n[종료]")
    finally:
        app.close()


def _draw_course(draw, target_idx):
    """H자 3획 라인 + 웨이포인트 점 + 현재 목표 강조."""
    z = 0.05
    p1 = [(s[0][0], s[0][1], z) for s in H_STROKES]
    p2 = [(s[1][0], s[1][1], z) for s in H_STROKES]
    cols = [(0.1, 0.8, 1.0, 1.0)] * len(H_STROKES)   # 하늘색 H
    sizes = [4.0] * len(H_STROKES)
    draw.clear_lines()
    draw.draw_lines(p1, p2, cols, sizes)

    pts = [(w[0], w[1], z) for w in H_WAYPOINTS]
    pcols = []
    for i in range(len(H_WAYPOINTS)):
        pcols.append((1.0, 0.9, 0.1, 1.0) if i == target_idx
                     else (1.0, 0.3, 0.3, 1.0))   # 목표=노랑, 나머지=빨강
    psizes = [12.0] * len(H_WAYPOINTS)
    draw.clear_points()
    draw.draw_points(pts, pcols, psizes)


if __name__ == "__main__":
    main()
