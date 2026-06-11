#!/usr/bin/env python3
# Copyright (c) 2024, Smart Farm Patrol Project - 꼬마 로봇 두리
# SPDX-License-Identifier: Apache-2.0
"""
add_sensors.py
==============
로봇 USD(spot_with_arm.usd)에 센서 prim 을 런타임으로 "붙이는" 코드.

spot_with_arm.usd 에는 센서가 없으므로, 로봇을 로드한 뒤 이 코드로
카메라 / LiDAR / IMU prim 을 로봇 하위에 생성합니다.
→ 이후 isaac_sim_bridge.py 의 센서 OmniGraph 가 이 prim 들을 ROS 2 로 발행.

생성 센서 (robot_prim 기준, 기본 /World/Spot):
  {robot}/arm0_link_fngr/RGBCamera          UsdGeom.Camera (90° FOV, 하향)
  {robot}/arm0_link_fngr/ThermalCamera      (열화상 비전 준비중 → 기본 생략)
  {robot}/base/Lidar360/lidar_sensor        RTX Lidar (Debug_Rotary, 360° 2D)
  {robot}/base/IMU                          IsaacSensor IMU (200Hz)

[두 가지 사용 방식]
  A) 단독 실행 — 로봇 + 센서를 GUI로 확인 (Stage 트리에 센서 prim 보임)
       source ~/dev_ws/venv/isaaclab/bin/activate
       cd ~/dev_ws/isaac_sim/IsaacLab
       ./isaaclab.sh -p ~/dev_ws/spot_ws/src/smart_farm_spot/isaac/add_sensors.py

  B) 모듈 import — 브릿지/배치 코드가 센서 생성 재사용
       import add_sensors
       add_sensors.add_sensors("/World/Spot")     # 로봇 로드 후 호출
"""

import math
import os

# ── 센서 스펙 (build_sensors_usd.py / spot_arm_sensor_cfg.py 와 일치) ──
RGB_OFFSET_POS   = (0.05, 0.0, -0.05)
RGB_OFFSET_ROT   = (90.0, 0.0, 0.0)    # 하향(검사용). 반대로 보이면 -90 으로 교체
RGB_FOCAL        = 12.0
RGB_APERTURE     = 20.955              # ≈90° FOV
RGB_CLIP         = (0.05, 10.0)

# ── 정면(주행용) 카메라 — base 앞쪽, +X(전방) 수평 시야 ──────────────
FRONT_OFFSET_POS    = (0.40, 0.0, 0.10)         # base 앞쪽 살짝 위
FRONT_ORIENT_WXYZ   = (0.5, 0.5, -0.5, -0.5)    # -Z_cam→+X(전방), +Y_cam→+Z(up)
FRONT_FOCAL         = 12.0
FRONT_APERTURE      = 24.0                       # ≈90° FOV (주행 시야)
FRONT_CLIP          = (0.05, 50.0)

# ── 뎁스 카메라 — 엔드이펙터(손, arm0_link_fngr) 하향, RGB 와 RGB-D 쌍 ──
# 소 유방/다리 근접 인지용. RGB 카메라와 같은 하향 자세.
DEPTH_OFFSET_POS    = (0.0, 0.0, -0.05)          # 손끝, RGB 옆
DEPTH_OFFSET_ROT    = (90.0, 0.0, 0.0)           # 하향 (RGB 와 동일)
DEPTH_FOCAL         = 12.0
DEPTH_APERTURE      = 20.955                      # ≈90° FOV
DEPTH_CLIP          = (0.05, 10.0)               # 뎁스 유효거리 ~10m

THM_OFFSET_POS   = (-0.05, 0.0, -0.05)
THM_OFFSET_ROT   = (-90.0, 0.0, 0.0)
THM_FOCAL        = 13.0
THM_APERTURE     = 12.7                # ≈45° FOV
THM_CLIP         = (0.05, 8.0)

LIDAR_MOUNT_Z    = 0.3                 # base 상단 +0.3m
# RTX 라이다 config = Nucleus 에셋 stem (로컬 JSON 아님).
# 유효 값: Example_Rotary_2D(2D 회전, Nav2용), Example_Rotary, OS1, OS0, Ouster_VLS_128 ...
LIDAR_CONFIG     = "Example_Rotary_2D"  # 2D 360° 회전 = Nav2 LaserScan 에 적합
IMU_PERIOD       = 1.0 / 200.0         # 200Hz


# ══════════════════════════════════════════════════════════════════════
# USD 헬퍼 (lazy import — SimulationApp/AppLauncher 생성 후 호출)
# ══════════════════════════════════════════════════════════════════════
def _make_camera(stage, path, translate, rotate_xyz, focal, aperture, clip,
                 orient_wxyz=None):
    """
    UsdGeom.Camera prim 생성 + 오프셋/광학 설정.
    방향은 rotate_xyz(도, XYZ) 또는 orient_wxyz(쿼터니언 w,x,y,z) 중 하나.
    """
    from pxr import UsdGeom, Gf
    cam = UsdGeom.Camera.Define(stage, path)
    cam.CreateFocalLengthAttr(float(focal))
    cam.CreateHorizontalApertureAttr(float(aperture))
    cam.CreateClippingRangeAttr(Gf.Vec2f(float(clip[0]), float(clip[1])))
    cam.CreateProjectionAttr("perspective")
    xf = UsdGeom.Xformable(cam.GetPrim())
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(*[float(v) for v in translate]))
    if orient_wxyz is not None:
        w, x, y, z = [float(v) for v in orient_wxyz]
        xf.AddOrientOp().Set(Gf.Quatf(w, Gf.Vec3f(x, y, z)))
    else:
        xf.AddRotateXYZOp().Set(Gf.Vec3f(*[float(v) for v in rotate_xyz]))
    return cam.GetPrim()


def _make_xform(stage, path, translate):
    """Xform prim 생성 + translate."""
    from pxr import UsdGeom, Gf
    xf = UsdGeom.Xform.Define(stage, path)
    UsdGeom.Xformable(xf).AddTranslateOp().Set(Gf.Vec3d(*[float(v) for v in translate]))
    return xf.GetPrim()


# ══════════════════════════════════════════════════════════════════════
# 센서 부착 (메인 진입점)
# ══════════════════════════════════════════════════════════════════════
def add_sensors(robot_prim="/World/Spot", enable_thermal=False,
                lidar_config=LIDAR_CONFIG):
    """
    로봇 하위에 카메라/LiDAR/IMU prim 을 생성.

    Args:
        robot_prim:    로봇 articulation prim 경로
        enable_thermal: 열화상 카메라도 생성할지 (기본 False — 비전 준비중)
        lidar_config:  RTX Lidar config 이름 (data/lidar_configs/.../<이름>.json)

    Returns:
        생성된 센서 경로 dict
    """
    import omni.usd
    import omni.kit.commands
    from isaacsim.core.utils.extensions import enable_extension

    # 센서 커맨드용 익스텐션 보장
    enable_extension("isaacsim.sensors.rtx")
    enable_extension("isaacsim.sensors.physics")

    stage = omni.usd.get_context().get_stage()
    fngr = f"{robot_prim}/arm0_link_fngr"
    base = f"{robot_prim}/base"

    print("=" * 60)
    print(f" [센서] 로봇에 센서 부착: {robot_prim}")
    print("=" * 60)

    # 부모 링크 존재 확인
    for link in (fngr, base):
        p = stage.GetPrimAtPath(link)
        if not (p and p.IsValid()):
            print(f"  ⚠️  링크 없음: {link}  (로봇이 로드됐는지 확인)")

    created = {}

    # ── 1. RGB Camera ─────────────────────────────────────────────
    rgb_path = f"{fngr}/RGBCamera"
    _make_camera(stage, rgb_path, RGB_OFFSET_POS, RGB_OFFSET_ROT,
                 RGB_FOCAL, RGB_APERTURE, RGB_CLIP)
    created["rgb"] = rgb_path
    print(f"  ✅ RGBCamera(arm 검사용) → {rgb_path}")

    # ── 1a. Depth Camera (엔드이펙터 손, 하향 — RGB-D 쌍) ──────────
    depth_path = f"{fngr}/DepthCamera"
    _make_camera(stage, depth_path, DEPTH_OFFSET_POS, DEPTH_OFFSET_ROT,
                 DEPTH_FOCAL, DEPTH_APERTURE, DEPTH_CLIP)
    created["depth"] = depth_path
    print(f"  ✅ DepthCamera(손끝 하향)  → {depth_path}")

    # ── 1b. Front Camera (주행용, 정면) ───────────────────────────
    front_path = f"{base}/FrontCamera"
    _make_camera(stage, front_path, FRONT_OFFSET_POS, None,
                 FRONT_FOCAL, FRONT_APERTURE, FRONT_CLIP,
                 orient_wxyz=FRONT_ORIENT_WXYZ)
    created["front"] = front_path
    print(f"  ✅ FrontCamera(주행용)   → {front_path}")

    # ── 2. Thermal Camera (선택) ──────────────────────────────────
    if enable_thermal:
        thm_path = f"{fngr}/ThermalCamera"
        _make_camera(stage, thm_path, THM_OFFSET_POS, THM_OFFSET_ROT,
                     THM_FOCAL, THM_APERTURE, THM_CLIP)
        created["thermal"] = thm_path
        print(f"  ✅ ThermalCamera → {thm_path}")
    else:
        print("  ⏸  ThermalCamera 생략 (비전 준비중)")

    # ── 3. LiDAR (Xform 마운트 + RTX lidar) ───────────────────────
    lidar_mount = f"{base}/Lidar360"
    _make_xform(stage, lidar_mount, (0.0, 0.0, LIDAR_MOUNT_Z))
    try:
        _, lidar_prim = omni.kit.commands.execute(
            "IsaacSensorCreateRtxLidar",
            path="/lidar_sensor",
            parent=lidar_mount,
            config=lidar_config,
        )
        created["lidar"] = f"{lidar_mount}/lidar_sensor"
        print(f"  ✅ RTX Lidar     → {lidar_mount}/lidar_sensor  (config={lidar_config})")
    except Exception as e:
        print(f"  ❌ RTX Lidar 생성 실패: {e}")

    # ── 4. IMU ────────────────────────────────────────────────────
    try:
        _, imu_prim = omni.kit.commands.execute(
            "IsaacSensorCreateImuSensor",
            path="/IMU",
            parent=base,
            sensor_period=IMU_PERIOD,
        )
        created["imu"] = f"{base}/IMU"
        print(f"  ✅ IMU           → {base}/IMU  ({1/IMU_PERIOD:.0f}Hz)")
    except Exception as e:
        print(f"  ❌ IMU 생성 실패: {e}")

    # ── 검증 ──────────────────────────────────────────────────────
    print("\n  [부착 검증]")
    for key, path in created.items():
        p = stage.GetPrimAtPath(path)
        ok = "✅" if (p and p.IsValid()) else "❌"
        print(f"    {ok} {key:<8} {path}")

    return created


# ══════════════════════════════════════════════════════════════════════
# 단독 실행 — 로봇 + 센서를 GUI 로 확인
# ══════════════════════════════════════════════════════════════════════
def main():
    import argparse
    parser = argparse.ArgumentParser(description="로봇에 센서 부착 (GUI 확인)")
    _assets = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "assets")
    parser.add_argument("--robot-usd", type=str,
                        default=os.path.join(_assets, "robot", "spot_with_arm.usd"))
    parser.add_argument("--robot-prim", type=str, default="/World/Spot")
    parser.add_argument("--spawn-z", type=float, default=0.65)
    parser.add_argument("--thermal", action="store_true", help="열화상 카메라도 생성")
    parser.add_argument("--lidar-config", type=str, default=LIDAR_CONFIG)
    parser.add_argument("--headless", action="store_true")
    a = parser.parse_args()

    # IsaacLab AppLauncher (bare SimulationApp 은 이 환경에서 omni.kit.usd 에러)
    from isaaclab.app import AppLauncher
    app = AppLauncher(headless=a.headless).app

    from isaacsim.core.api import World
    from isaacsim.core.utils.stage import add_reference_to_stage
    import isaacsim.core.utils.prims as prim_utils

    print("=" * 60)
    print(" 로봇 + 센서 부착 (GUI 확인)")
    print("=" * 60)

    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()

    if not os.path.exists(a.robot_usd):
        print(f"❌ 로봇 USD 없음: {a.robot_usd}")
        app.close()
        return
    add_reference_to_stage(usd_path=a.robot_usd, prim_path=a.robot_prim)
    prim_utils.set_prim_attribute_value(
        a.robot_prim, "xformOp:translate", [0.0, 0.0, a.spawn_z])
    app.update()

    # ── 센서 부착 ─────────────────────────────────────────────────
    add_sensors(a.robot_prim, enable_thermal=a.thermal, lidar_config=a.lidar_config)

    world.reset()
    app.update()

    print("\n[보기] Stage 트리에서 RGBCamera / Lidar360 / IMU 확인.")
    print("       Play(▶) 로 LiDAR 스캔 동작. (창 닫기/Ctrl+C 종료)\n")

    try:
        while app.is_running():
            world.step(render=True)
    except KeyboardInterrupt:
        print("\n[종료]")
    finally:
        app.close()


if __name__ == "__main__":
    main()
