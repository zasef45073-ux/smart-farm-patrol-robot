#!/usr/bin/env python3
# Copyright (c) 2024, Smart Farm Patrol Project - 꼬마 로봇 두리
# SPDX-License-Identifier: Apache-2.0
"""
record_4cam.py
==============
**헤드리스 4뷰 동시 녹화** — 한 시나리오를 4개 카메라로 mp4 저장(지정 폴더).

4개 뷰:
  1. barn_oblique : 축사 사선(고정 오블리크) — 전경 부감
  2. robot_chase  : 로봇 추종 체이스 캠(매 프레임 로봇 뒤·위에서 따라감)
  3. hand_cam     : 손 끝단(arm0_link_fngr) RealSense RGB — 검사 시점
  4. top_map      : 탑다운(부감) 맵 뷰 — 전체 배치/경로 확인

로봇은 정책(spot_flat) 보행으로 전진 순찰(SF_REC_VX/WZ)하며 4뷰를 동시 녹화한다.
camera_record.py 의 검증된 환경 셋업(축사·구역마찰·가상벽·안착)을 재사용.

저장 폴더(지정 가능):
  SF_OUT_DIR 로 지정 — 없으면 ~/rag/<YYYYMMDD_HHMMSS>_4cam/
  결과: barn_oblique.mp4 / robot_chase.mp4 / hand_cam.mp4 / top_map.mp4

실행(헤드리스):
  source ~/dev_ws/venv/isaaclab/bin/activate
  cd ~/dev_ws/isaac_sim/IsaacLab
  SF_HEADLESS=1 SF_OUT_DIR=~/rag/demo1 \
    ./isaaclab.sh -p ~/dev_ws/spot_ws/src/smart_farm_spot/isaac/record_4cam.py
"""

import argparse
import datetime
import math
import os

os.environ["ROS_DOMAIN_ID"] = os.environ.get("ROS_DOMAIN_ID", "153")

_ASSETS = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "assets")
TASK = "Isaac-Velocity-Rough-SpotArm-Play-v0"
POLICY = os.environ.get("SF_POLICY", os.path.join(_ASSETS, "policy", "policy.pt"))
ENV_USD = os.path.join(_ASSETS, "scene", "environment_final.usd")
HOME = os.path.expanduser("~")

RES = tuple(int(v) for v in os.environ.get("SF_REC_RES", "640,480").split(","))  # (W,H)
SECONDS = float(os.environ.get("SF_REC_SECONDS", "30"))
FPS = int(os.environ.get("SF_REC_FPS", "30"))
LOAD_BARN = os.environ.get("SF_BARN", "1") == "1"
PEN_FRICTION = float(os.environ.get("SF_PEN_FRICTION", "0.45"))
CORRIDOR_FRICTION = float(os.environ.get("SF_CORRIDOR_FRICTION", "0.7"))

# 순찰 보행 명령(정책 입력) — 전진 vx, 회전 wz (base frame)
REC_VX = float(os.environ.get("SF_REC_VX", "0.5"))
REC_WZ = float(os.environ.get("SF_REC_WZ", "0.0"))

# 카메라 배치 파라미터(축사 규모에 맞춰 env 로 튜닝 가능)
OBQ_OFF = float(os.environ.get("SF_OBQ_OFF", "9.0"))    # 오블리크 수평 오프셋
OBQ_H = float(os.environ.get("SF_OBQ_H", "7.0"))        # 오블리크 높이
CHASE_BACK = float(os.environ.get("SF_CHASE_BACK", "3.0"))  # 체이스 뒤 거리
CHASE_UP = float(os.environ.get("SF_CHASE_UP", "1.8"))      # 체이스 높이
MAP_H = float(os.environ.get("SF_MAP_H", "24.0"))      # 탑다운 높이

# 과노출 완화: 캡처 픽셀 게인(<1=어둡게) — 조명 감광 후에도 밝으면 추가로 낮춤(확실)
EXPOSURE_GAIN = float(os.environ.get("SF_EXPOSURE_GAIN", "0.6"))
# 손 카메라 시점 틸트(deg, +면 위로) — 바닥만 보던 것 전방/소 높이로
HAND_PITCH_DEG = float(os.environ.get("SF_HAND_PITCH_DEG", "25.0"))

_TS = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
OUT_DIR = os.environ.get("SF_OUT_DIR", os.path.join(HOME, "rag", f"{_TS}_4cam"))

parser = argparse.ArgumentParser()
parser.add_argument("--task", default=TASK)
parser.add_argument("--policy", default=POLICY)
a = parser.parse_args()

from isaaclab.app import AppLauncher  # noqa: E402
_HEADLESS = os.environ.get("SF_HEADLESS", "1") == "1"   # 기본 헤드리스
app_launcher = AppLauncher(headless=_HEADLESS, enable_cameras=True)
simulation_app = app_launcher.app

from isaacsim.core.utils.extensions import enable_extension  # noqa: E402
for _ext in ("omni.replicator.core",):
    try:
        enable_extension(_ext)
    except Exception as _e:
        print(f"  ⚠️ ext {_ext}: {_e}")
simulation_app.update()

import numpy as np  # noqa: E402
import torch  # noqa: E402
import cv2  # noqa: E402
import gymnasium as gym  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
import omni.replicator.core as rep  # noqa: E402
from pxr import Gf, Usd, UsdGeom, UsdPhysics, UsdShade  # noqa: E402
from isaacsim.core.utils.stage import add_reference_to_stage  # noqa: E402
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR  # noqa: E402
import isaaclab_tasks  # noqa: F401,E402
from isaaclab_tasks.utils import parse_env_cfg  # noqa: E402


# ── 수학 헬퍼: 회전행렬 → quat, look-at quat ─────────────────────────
def _mat_to_quat(R):
    """3x3 회전행렬(월드←카메라) → Gf.Quatf(w,x,y,z)."""
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return Gf.Quatf(float(w), Gf.Vec3f(float(x), float(y), float(z)))


def _look_at_quat(eye, target, up=(0.0, 0.0, 1.0)):
    """eye 에서 target 을 바라보는 USD 카메라 orient(−Z 전방, +Y up)."""
    eye = np.array(eye, float)
    target = np.array(target, float)
    up = np.array(up, float)
    back = eye - target                       # 카메라 +Z (피사체 반대)
    back /= (np.linalg.norm(back) + 1e-9)
    right = np.cross(up, back)
    if np.linalg.norm(right) < 1e-6:          # up∥back (탑다운) → 대체 up
        up = np.array([0.0, 1.0, 0.0])
        right = np.cross(up, back)
    right /= (np.linalg.norm(right) + 1e-9)
    upv = np.cross(back, right)               # 카메라 +Y
    R = np.column_stack([right, upv, back])   # 월드←카메라
    return _mat_to_quat(R)


def _make_camera(stage, path, focal=18.0, aperture=24.0, clip=(0.05, 2000.0)):
    """월드 UsdGeom.Camera 생성(+translate/orient op 준비)."""
    cam = UsdGeom.Camera.Define(stage, path)
    cam.GetFocalLengthAttr().Set(float(focal))
    cam.GetHorizontalApertureAttr().Set(float(aperture))
    cam.GetClippingRangeAttr().Set(Gf.Vec2f(float(clip[0]), float(clip[1])))
    xf = UsdGeom.Xformable(cam.GetPrim())
    xf.ClearXformOpOrder()
    t_op = xf.AddTranslateOp()
    o_op = xf.AddOrientOp()
    return cam, t_op, o_op


def _set_pose(t_op, o_op, eye, quat):
    t_op.Set(Gf.Vec3d(float(eye[0]), float(eye[1]), float(eye[2])))
    o_op.Set(quat)


# ── 환경 셋업 헬퍼(camera_record.py 검증판 재사용) ───────────────────
def apply_zone_friction(stage, barn_root, pen_mu, corridor_mu):
    root = stage.GetPrimAtPath(barn_root)
    if not root or not root.IsValid():
        return 0

    def _mat(path, mu):
        m = UsdShade.Material.Define(stage, path)
        UsdPhysics.MaterialAPI.Apply(m.GetPrim())
        pm = UsdPhysics.MaterialAPI(m.GetPrim())
        pm.CreateStaticFrictionAttr(mu)
        pm.CreateDynamicFrictionAttr(mu * 0.9)
        pm.CreateRestitutionAttr(0.0)
        return m

    pen_mat = _mat("/World/PenMat", pen_mu)
    cor_mat = _mat("/World/CorridorMat", corridor_mu)
    n = 0
    for p in Usd.PrimRange(root):
        if p.GetTypeName() != "Mesh":
            continue
        nm = p.GetName().lower()
        if "bedding" in nm:
            m = pen_mat
        elif ("sand" in nm and "corridor" in nm) or nm == "ground_mesh" or "floor" in nm:
            m = cor_mat
        else:
            continue
        UsdPhysics.CollisionAPI.Apply(p)
        b = UsdShade.MaterialBindingAPI.Apply(p)
        b.Bind(m, bindingStrength=UsdShade.Tokens.strongerThanDescendants,
               materialPurpose="physics")
        n += 1
    print(f"  ✅ 구역마찰 적용: {n} mesh")
    return n


def apply_wall_collision(stage, barn_root):
    root = stage.GetPrimAtPath(barn_root)
    if not root or not root.IsValid():
        return 0
    kws = ("barrier", "escape", "fence", "wall", "rail")
    geom = ("Mesh", "Cube", "Cylinder", "Capsule", "Cone", "Sphere")
    n = 0
    for p in Usd.PrimRange(root):
        path = str(p.GetPath()).lower()
        if any(k in path for k in kws) and p.GetTypeName() in geom:
            UsdPhysics.CollisionAPI.Apply(p)
            n += 1
    print(f"  ✅ 가상벽/펜스 콜라이더: {n}개")
    return n


def dim_lights(stage, scale):
    """씬의 모든 UsdLux 라이트 intensity 를 scale 배로 낮춤 → **과노출(흰화면) 완화**.
      축사 USD 자체 조명 + IsaacLab 기본광이 합쳐져 과노출 → 일괄 감광.
    """
    from pxr import UsdLux
    n = 0
    for p in stage.Traverse():
        if "Light" not in p.GetTypeName():
            continue
        try:
            a = UsdLux.LightAPI(p).GetIntensityAttr()
            if a and a.Get() is not None:
                a.Set(float(a.Get()) * scale)
                n += 1
        except Exception:
            pass
    print(f"  ✅ 조명 감광 ×{scale}: {n}개 라이트")
    return n


def mount_hand_realsense(stage, robot_prim):
    """손 끝단에 RealSense 부착 → 컬러 카메라 prim 경로 반환(camera_record 방식 축약)."""
    _ee = None
    for _pref in ("arm0_link_fngr", "arm0_link_wr1", "arm0_link_wr0"):
        for _p in stage.Traverse():
            if _p.GetName() == _pref:
                _ee = str(_p.GetPath())
                break
        if _ee:
            break
    _ee = _ee or robot_prim
    mount = f"{_ee}/RealSense4Cam"
    add_reference_to_stage(
        usd_path=f"{ISAAC_NUCLEUS_DIR}/Sensors/Intel/RealSense/rsd455.usd",
        prim_path=mount)
    # 손등 위 오프셋(월드 +Z 약간) → 로컬
    _W = np.array([0.0, 0.0, 0.06])
    _Mf = UsdGeom.Xformable(stage.GetPrimAtPath(_ee)).ComputeLocalToWorldTransform(
        Usd.TimeCode.Default())
    _RM = _Mf.ExtractRotationMatrix()
    _Rf = np.array([[_RM[i][j] for j in range(3)] for i in range(3)], float)
    _loc = _W @ _Rf.T
    mx = UsdGeom.Xformable(stage.GetPrimAtPath(mount))
    mx.ClearXformOpOrder()
    mx.AddTranslateOp().Set(Gf.Vec3d(float(_loc[0]), float(_loc[1]), float(_loc[2])))
    o_op = mx.AddOrientOp()
    o_op.Set(Gf.Quatf(1.0, Gf.Vec3f(0.0, 0.0, 0.0)))
    simulation_app.update()

    color = f"{mount}/RSD455/Camera_OmniVision_OV9782_Color"
    # 광축 자동정렬(컬러 -Z/+Y → +X 전방/+Z up) — camera_record 와 동일 로직 축약
    _M = UsdGeom.Camera(stage.GetPrimAtPath(color)).ComputeLocalToWorldTransform(
        Usd.TimeCode.Default())
    _R = _M.ExtractRotationMatrix()
    _f = np.array([-_R[2][0], -_R[2][1], -_R[2][2]], float)
    _u = np.array([_R[1][0], _R[1][1], _R[1][2]], float)
    _f /= (np.linalg.norm(_f) + 1e-9)
    _u /= (np.linalg.norm(_u) + 1e-9)
    _Mc = np.column_stack([_f, _u, np.cross(_f, _u)])
    _Mt = np.column_stack([[1., 0, 0], [0, 0, 1.], [0, -1., 0]])
    _Rm = _Mt @ _Mc.T
    # 손카메라 시점 틸트 — 바닥만 보던 것 위로(전방/소 높이). 카메라 X축 기준 피치.
    if abs(HAND_PITCH_DEG) > 1e-3:
        _th = math.radians(HAND_PITCH_DEG)
        _Rp = np.array([[1.0, 0.0, 0.0],
                        [0.0, math.cos(_th), -math.sin(_th)],
                        [0.0, math.sin(_th), math.cos(_th)]], float)
        _Rm = _Rm @ _Rp
    o_op.Set(_mat_to_quat(_Rm))
    c = stage.GetPrimAtPath(color)
    if c and c.IsValid():
        UsdGeom.Camera(c).GetClippingRangeAttr().Set(Gf.Vec2f(0.01, 1000.0))
    print(f"  ✅ 손 RealSense 마운트: {color}")
    return color


def _yaw_from_quat(q):
    """root_quat_w(w,x,y,z) → yaw(rad)."""
    w, x, y, z = float(q[0]), float(q[1]), float(q[2]), float(q[3])
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def main():
    print("=" * 64)
    print(" 헤드리스 4뷰 녹화 (barn_oblique / robot_chase / hand_cam / top_map)")
    print(f"   {SECONDS:.0f}s @ {FPS}fps, {RES[0]}x{RES[1]} → {OUT_DIR}")
    print("=" * 64)

    env_cfg = parse_env_cfg(a.task, device="cuda:0", num_envs=1)
    try:
        for _n in [n for n in vars(env_cfg.terminations) if not n.startswith("_")]:
            setattr(env_cfg.terminations, _n, None)
        print("  ✅ 자동 리셋 끔")
    except Exception as e:
        print(f"  ⚠️ termination: {e}")

    if os.environ.get("SF_ROUGH", "0") != "1":
        env_cfg.scene.terrain.terrain_type = "plane"
        env_cfg.scene.terrain.terrain_generator = None
        try:
            env_cfg.curriculum.terrain_levels = None
        except Exception:
            pass
        try:
            env_cfg.scene.height_scanner.mesh_prim_paths = ["/World/ground"]
        except Exception:
            pass
        try:
            env_cfg.events.physics_material = None
        except Exception:
            pass
        print("  ✅ 평면 + 구역마찰 모드")

    env = gym.make(a.task, cfg=env_cfg)
    obs, _ = env.reset()
    device = env.unwrapped.device
    stage = omni.usd.get_context().get_stage()
    robot = env.unwrapped.scene["robot"]
    robot_prim = robot.root_physx_view.prim_paths[0]

    # 팔 경량화(massless 트릭) — 무팔 보행정책 CoM 외란 제거 → **뒤집힘 방지**.
    #  arm_mass.py 는 패키지 루트에 있음 → sys.path 추가 후 import.
    _ARM_LIGHT = float(os.environ.get("SF_ARM_LIGHT_KG", "0.05"))
    if _ARM_LIGHT > 0:
        import sys as _sys
        _root = os.path.dirname(_ASSETS)
        if _root not in _sys.path:
            _sys.path.insert(0, _root)
        try:
            from arm_mass import apply_light_arm
            _n = apply_light_arm(robot, _ARM_LIGHT)
            print(f"  ✅ 팔 경량화 {_n}개 링크 → {_ARM_LIGHT}kg (뒤집힘 방지)")
        except Exception as e:
            print(f"  ⚠️ 팔 경량화: {e}")

    policy = torch.jit.load(a.policy, map_location=device)
    policy.eval()
    cmd_term = env.unwrapped.command_manager.get_term("base_velocity")
    try:
        cmd_term.cfg.resampling_time_range = (1.0e9, 1.0e9)
        cmd_term.cfg.heading_command = False
        cmd_term.vel_command_b[:] = 0.0
    except Exception:
        pass

    # 워밍업(공중 스폰 → 발 안착)
    for _ in range(60):
        with torch.inference_mode():
            _o = obs["policy"] if isinstance(obs, dict) else obs
            obs, _, _, _, _ = env.step(policy(_o))

    _rs = robot.data.root_state_w[0]
    SX, SY, SZ = float(_rs[0]), float(_rs[1]), float(_rs[2])
    try:
        _fz = float(robot.data.body_pos_w[0, :, 2].min())
    except Exception:
        _fz = SZ - 0.55
    FLOOR_Z = _fz - float(os.environ.get("SF_FOOT_OFFSET", "0.20"))
    print(f"  ✅ 로봇 안착: ({SX:.2f},{SY:.2f},{SZ:.2f}) → 바닥 z={FLOOR_Z:.3f}")

    # 축사 배경
    if LOAD_BARN and os.path.exists(ENV_USD):
        try:
            add_reference_to_stage(usd_path=ENV_USD, prim_path="/World/Barn")
            _bx = UsdGeom.Xformable(stage.GetPrimAtPath("/World/Barn"))
            _bx.ClearXformOpOrder()
            _bx.AddTranslateOp().Set(Gf.Vec3d(SX, SY, FLOOR_Z))
            simulation_app.update()
            print("  ✅ 축사 배경 로드")
            if os.environ.get("SF_ROUGH", "0") != "1":
                apply_zone_friction(stage, "/World/Barn", PEN_FRICTION, CORRIDOR_FRICTION)
            apply_wall_collision(stage, "/World/Barn")
        except Exception as e:
            print(f"  ⚠️ 축사: {e}")

    if os.environ.get("SF_HIDE_GRID", "1") == "1":
        for _gp in ("/World/ground", "/World/GroundPlane", "/World/defaultGroundPlane"):
            _g = stage.GetPrimAtPath(_gp)
            if _g and _g.IsValid():
                try:
                    UsdGeom.Imageable(_g).MakeInvisible()
                except Exception:
                    pass

    # 과노출 완화 — 씬 조명 일괄 감광(SF_LIGHT_SCALE, 0=끄지않음)
    _LSCALE = float(os.environ.get("SF_LIGHT_SCALE", "0.35"))
    if _LSCALE > 0:
        dim_lights(stage, _LSCALE)

    # ── 4개 카메라 생성 ──────────────────────────────────────────────
    # 1. 축사 사선(고정 오블리크)
    obq_cam, obq_t, obq_o = _make_camera(stage, "/World/Cam_BarnOblique",
                                         focal=18.0, aperture=28.0)
    obq_eye = (SX + OBQ_OFF, SY - OBQ_OFF, FLOOR_Z + OBQ_H)
    _set_pose(obq_t, obq_o, obq_eye,
              _look_at_quat(obq_eye, (SX, SY, FLOOR_Z + 0.5)))

    # 2. 로봇 체이스(매 프레임 갱신)
    chase_cam, chase_t, chase_o = _make_camera(stage, "/World/Cam_RobotChase",
                                               focal=20.0, aperture=24.0)

    # 3. 손 RealSense
    hand_color = mount_hand_realsense(stage, robot_prim)

    # 4. 탑다운 맵(부감) — 넓게 보이도록 wide
    map_cam, map_t, map_o = _make_camera(stage, "/World/Cam_TopMap",
                                         focal=12.0, aperture=36.0)
    map_eye = (SX, SY, FLOOR_Z + MAP_H)
    _set_pose(map_t, map_o, map_eye, _look_at_quat(map_eye, (SX, SY, FLOOR_Z)))

    simulation_app.update()

    # ── 렌더프로덕트 + annotator (뷰별) ─────────────────────────────
    cams = [
        ("barn_oblique", "/World/Cam_BarnOblique"),
        ("robot_chase", "/World/Cam_RobotChase"),
        ("hand_cam", hand_color),
        ("top_map", "/World/Cam_TopMap"),
    ]
    views = []
    for name, path in cams:
        rp = rep.create.render_product(path, RES)
        an = rep.AnnotatorRegistry.get_annotator("rgb")
        an.attach(rp)
        views.append({"name": name, "rp": rp, "an": an})
    print(f"  ✅ 렌더프로덕트 4개 ({RES[0]}x{RES[1]})")

    omni.timeline.get_timeline_interface().play()
    simulation_app.update()

    # ── 비디오 라이터 4개 ───────────────────────────────────────────
    os.makedirs(OUT_DIR, exist_ok=True)
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    for v in views:
        v["vw"] = cv2.VideoWriter(
            os.path.join(OUT_DIR, f"{v['name']}.mp4"), fourcc, FPS, RES)
        v["written"] = 0

    # 순찰 보행 명령(전진/회전)
    try:
        cmd_term.vel_command_b[:, 0] = REC_VX
        if cmd_term.vel_command_b.shape[1] > 2:
            cmd_term.vel_command_b[:, 2] = REC_WZ
    except Exception as e:
        print(f"  ⚠️ 명령 설정: {e}")
    print(f"  ✅ 순찰 보행: vx={REC_VX} wz={REC_WZ}")

    n_frames = int(SECONDS * FPS)
    print(f"\n[REC START] {n_frames} 프레임 → {OUT_DIR}\n")

    for i in range(n_frames):
        with torch.inference_mode():
            actions = policy(obs["policy"] if isinstance(obs, dict) else obs)
            obs, _, _, _, _ = env.step(actions)
            try:
                cmd_term.vel_command_b[:, 0] = REC_VX   # 명령 유지
                if cmd_term.vel_command_b.shape[1] > 2:
                    cmd_term.vel_command_b[:, 2] = REC_WZ
            except Exception:
                pass

        # 체이스 캠: 로봇 뒤·위에서 추종
        rp_ = robot.data.root_state_w[0]
        rx, ry, rz = float(rp_[0]), float(rp_[1]), float(rp_[2])
        yaw = _yaw_from_quat(robot.data.root_quat_w[0])
        hx, hy = math.cos(yaw), math.sin(yaw)
        chase_eye = (rx - hx * CHASE_BACK, ry - hy * CHASE_BACK, rz + CHASE_UP)
        _set_pose(chase_t, chase_o, chase_eye, _look_at_quat(chase_eye, (rx, ry, rz + 0.3)))

        simulation_app.update()   # 렌더 갱신

        for v in views:
            rgb = v["an"].get_data()
            if rgb is not None and getattr(rgb, "size", 0) > 0:
                arr = np.asarray(rgb)
                if arr.ndim == 3 and arr.shape[2] >= 3:
                    bgr = cv2.cvtColor(arr[:, :, :3].astype(np.uint8), cv2.COLOR_RGB2BGR)
                    if EXPOSURE_GAIN != 1.0:   # 과노출 완화(픽셀 게인)
                        bgr = np.clip(bgr.astype(np.float32) * EXPOSURE_GAIN,
                                      0, 255).astype(np.uint8)
                    if bgr.shape[:2] != (RES[1], RES[0]):
                        bgr = cv2.resize(bgr, RES)
                    v["vw"].write(bgr)
                    v["written"] += 1
        if (i + 1) % FPS == 0:
            print(f"  ... {i + 1}/{n_frames} 프레임 "
                  + " ".join(f"{v['name']}={v['written']}" for v in views))

    for v in views:
        v["vw"].release()
    print(f"\n✅ 4뷰 녹화 완료 → {OUT_DIR}")
    for v in views:
        print(f"   {v['name']:13s}: {os.path.join(OUT_DIR, v['name'] + '.mp4')} "
              f"({v['written']} 프레임)")
    env.close()
    simulation_app.close()


if __name__ == "__main__":
    main()
