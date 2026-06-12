#!/usr/bin/env python3
# Copyright (c) 2024, Smart Farm Patrol Project - 꼬마 로봇 두리
# SPDX-License-Identifier: Apache-2.0
"""
scenario.py
===========
순찰 → 소 발견 → 후방(꼬리 쪽) 이동 시나리오.

토대(camera_record 검증분):
  · 평면 + 안정 평면정책(spot_flat) → 안정 보행, 축사 바닥 정렬
  · 축사: 구역별 마찰(우리 0.45 / 통로 0.7) + 가상벽 콜라이더(통과 방지)
  · 손 RealSense RGB+Depth(320x320) → ROS 발행(YOLO 검출용)
추가:
  · 텍스처 소(cow_eat.usd) 스폰 — YOLO 검출 대상
  · go-to-goal 방향제어(eval_h_course_nav 방식): (vx, wz) 로 목표 지향 보행
  · 상태기계: PATROL(순찰) → APPROACH_REAR(후방 접근) → INSPECT(검사 정지)

실행:
  source ~/dev_ws/venv/isaaclab/bin/activate
  cd ~/dev_ws/isaac_sim/IsaacLab
  SF_POLICY=.../spot_flat/2026-06-07_14-01-53/exported/policy.pt \
    ./isaaclab.sh -p ~/dev_ws/spot_ws/src/smart_farm_spot/isaac/scenario.py
  # 다른 터미널(시스템 ROS2, 도메인153): python3 yolo_view.py
"""

import argparse
import math
import os

os.environ["ROS_DOMAIN_ID"] = os.environ.get("ROS_DOMAIN_ID", "153")

_ASSETS = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "assets")
TASK = "Isaac-Velocity-Rough-SpotArm-Play-v0"
POLICY = os.environ.get("SF_POLICY", os.path.join(_ASSETS, "policy", "policy.pt"))
ENV_USD = os.path.join(_ASSETS, "scene", "environment_final.usd")
COW_USD = os.path.join(_ASSETS, "scene", "models", "cow_eat.usd")
COW_TEX = os.path.join(_ASSETS, "scene", "textures", "cow_fbx",
                       "RSG_Shepherd_Pack_FBX", "Cow", "Textures", "T_Cow_B.png")

RES = tuple(int(v) for v in os.environ.get("SF_CAM_RES", "640,640").split(","))  # 검출 신뢰도↑
PEN_FRICTION = float(os.environ.get("SF_PEN_FRICTION", "0.45"))
CORRIDOR_FRICTION = float(os.environ.get("SF_CORRIDOR_FRICTION", "0.7"))

# 소: 로봇 시작(원점) 기준 상대 월드좌표 + 바라보는 yaw(도)
COW_DX, COW_DY = (float(v) for v in os.environ.get("SF_COW_XY", "3.0,0.0").split(","))
COW_YAW = math.radians(float(os.environ.get("SF_COW_YAW", "0")))   # +x 향함
# 소 마릿수: 1=데모(기본), 5=시나리오(간격 배치). SF_COW_SPACING=옆 간격(m).
COW_COUNT = int(os.environ.get("SF_COW_COUNT", "1"))
COW_SPACING = float(os.environ.get("SF_COW_SPACING", "2.5"))
REAR_DIST = float(os.environ.get("SF_REAR_DIST", "2.4"))           # 후방 정차 거리(m) — 소 몸통(~3.2m) 밖
DETECT_RANGE = float(os.environ.get("SF_DETECT_RANGE", "2.2"))     # 소 발견 반경(m)
SPEED = float(os.environ.get("SF_SPEED", "0.9"))                   # 최대 전진(m/s)
GOAL_R = float(os.environ.get("SF_GOAL_R", "0.45"))               # 도착 판정(m)
FALL_Z = float(os.environ.get("SF_FALL_Z", "0.30"))               # base z 이하면 넘어짐
STAND_Z = float(os.environ.get("SF_STAND_Z", "0.62"))             # 강제 기립 높이
# 후방 도착 → 앉아서 검사 (실험적). 1=활성. 외부(scenario_nav)가 도착 시 트리거파일 생성.
INSPECT_ENABLE = int(os.environ.get("SF_INSPECT_ENABLE", "0"))
INSPECT_TRIGGER = os.environ.get("SF_INSPECT_TRIGGER", "/tmp/scenario_inspect")
REC_FORCE = int(os.environ.get("SF_REC_FORCE", "90"))            # 자력복구 실패 시 강제기립 스텝
STUCK_WIN = int(os.environ.get("SF_STUCK_WIN", "100"))           # 끼임 판정 윈도(스텝)
STUCK_DIST = float(os.environ.get("SF_STUCK_DIST", "0.25"))     # 윈도 내 이동 이하면 끼임
BACKUP_DUR = int(os.environ.get("SF_BACKUP_DUR", "70"))         # 후진 탈출 스텝

# ── in-Isaac 비전 꼬리 검출(SF_VISION_TAIL=1): YOLO·꼬리3D를 프로세스 내부에서 처리,
#    밖으론 목표(/tmp/scenario_goal.json)만 내보냄 → scenario_nav.py 가 Nav2 로 relay ──
VISION_TAIL = os.environ.get("SF_VISION_TAIL", "0") == "1"
DET_CONF = float(os.environ.get("SF_DET_CONF", "0.55"))        # 오검출↓ 위해 상향
KPT_CONF = float(os.environ.get("SF_KPT_CONF", "0.30"))
MIN_BBOX_FRAC = float(os.environ.get("SF_MIN_BBOX_FRAC", "0.28"))  # 소 bbox 높이/이미지 최소(가까울 때만 신뢰)
TAIL_KPT = 5                                                    # 0 nose..5 tail_base
N_STABLE = int(os.environ.get("SF_N_STABLE", "5"))             # 안정 추정 측정 수
RESEND_DIST = float(os.environ.get("SF_RESEND_DIST", "0.6"))   # 목표 갱신 임계(m)
VIS_EVERY = int(os.environ.get("SF_VIS_EVERY", "15"))          # N스텝마다 YOLO 추론
SAVE_DET = os.environ.get("SF_SAVE_DET", "0") == "1"           # 640 검출 출력 저장
SAVE_DET_PATH = os.environ.get("SF_SAVE_DET_PATH",
                               os.path.expanduser("~/spot_det_640.jpg"))
# ── cmd_vel 추종 PID(하부 속도 폐루프): Nav2 목표속도를 실측 base 속도로 추종 ──
CMD_PID = os.environ.get("SF_CMD_PID", "1") == "1"
PID_KP = float(os.environ.get("SF_PID_KP", "0.6"))            # 선속도 P
PID_KI = float(os.environ.get("SF_PID_KI", "0.3"))            # 선속도 I
PID_KD = float(os.environ.get("SF_PID_KD", "0.0"))            # 선속도 D
PID_KP_W = float(os.environ.get("SF_PID_KP_W", "0.5"))       # 각속도 P
PID_KI_W = float(os.environ.get("SF_PID_KI_W", "0.2"))       # 각속도 I
PID_I_CLAMP = float(os.environ.get("SF_PID_I_CLAMP", "0.5")) # 적분 windup 한계
PID_VX_LIM = float(os.environ.get("SF_PID_VX_LIM", "1.2"))   # 보정후 vx 한계
PID_WZ_LIM = float(os.environ.get("SF_PID_WZ_LIM", "1.5"))   # 보정후 wz 한계
STABLE_WAIT = os.environ.get("SF_STABLE_WAIT", "1") == "1"   # 시작 대기 중 베이스 고정(안정 기립)

parser = argparse.ArgumentParser()
parser.add_argument("--task", default=TASK)
parser.add_argument("--policy", default=POLICY)
a = parser.parse_args()

from isaaclab.app import AppLauncher  # noqa: E402
_HEADLESS = os.environ.get("SF_HEADLESS", "0") == "1"
app_launcher = AppLauncher(headless=_HEADLESS, enable_cameras=True)
simulation_app = app_launcher.app

from isaacsim.core.utils.extensions import enable_extension  # noqa: E402
for _ext in ("omni.graph.core", "omni.graph.action", "omni.graph.nodes",
             "isaacsim.core.nodes", "isaacsim.ros2.bridge", "omni.replicator.core"):
    try:
        enable_extension(_ext)
    except Exception as _e:
        print(f"  ⚠️ ext {_ext}: {_e}")
simulation_app.update()

import numpy as np  # noqa: E402
import torch  # noqa: E402
import gymnasium as gym  # noqa: E402
import omni.graph.core as og  # noqa: E402
import omni.kit.commands  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
import omni.replicator.core as rep  # noqa: E402
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade  # noqa: E402
from isaacsim.core.utils.stage import add_reference_to_stage  # noqa: E402
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR  # noqa: E402
from isaaclab.utils.math import (  # noqa: E402
    euler_xyz_from_quat, wrap_to_pi,
    quat_mul, quat_inv, quat_apply, quat_apply_inverse)
import isaaclab_tasks  # noqa: F401,E402
from isaaclab_tasks.utils import parse_env_cfg  # noqa: E402

NS_ROS2 = "isaacsim.ros2.bridge"
NS_CORE = "isaacsim.core.nodes"
GRAPH = "/ScenarioGraph"
RS_SUB = "RSD455"
RS_COLOR_REL = f"{RS_SUB}/Camera_OmniVision_OV9782_Color"
RS_DEPTH_REL = f"{RS_SUB}/Camera_Pseudo_Depth"
LIDAR_MOUNT_Z = float(os.environ.get("SF_LIDAR_Z", "0.55"))  # 라이다 높이(0.35→0.55, 펜스 위로 — env 조정)
LIDAR_CONFIG = "Example_Rotary_2D"
TOPIC_SCAN = "/scan"
FRAME_LIDAR = "lidar_frame"


# ─────────────────────────── 축사 마찰/벽/소 ──────────────────────────
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
    print(f"  ✅ 구역마찰 {n}개 (우리 {pen_mu}/통로 {corridor_mu})")
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
    print(f"  ✅ 가상벽/펜스 콜라이더 {n}개 (통과 방지)")
    return n


def find_corridor_end(stage, barn_root, rx, ry):
    """Sand 통로들이 **겹치는 교차점(삼거리) 중앙** 반환. 로봇에서 가장 가까운 교차점.
    교차 없으면 가장 긴 통로 중앙으로 폴백."""
    root = stage.GetPrimAtPath(barn_root)
    if not root or not root.IsValid():
        return None
    bbc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    boxes = []
    for p in Usd.PrimRange(root):
        nm = p.GetName().lower()
        if p.GetTypeName() == "Mesh" and "sand" in nm and "corridor" in nm:
            r = bbc.ComputeWorldBound(p).ComputeAlignedRange()
            boxes.append((r.GetMin(), r.GetMax()))
    if not boxes:
        return None
    # 통로 겹침 = 삼거리(교차) 중앙들
    junctions = []
    for i in range(len(boxes)):
        for j in range(i + 1, len(boxes)):
            (mi, xi), (mj, xj) = boxes[i], boxes[j]
            ox0, ox1 = max(float(mi[0]), float(mj[0])), min(float(xi[0]), float(xj[0]))
            oy0, oy1 = max(float(mi[1]), float(mj[1])), min(float(xi[1]), float(xj[1]))
            if ox1 > ox0 and oy1 > oy0:
                junctions.append(((ox0 + ox1) / 2.0, (oy0 + oy1) / 2.0))
    if junctions:
        sel = int(os.environ.get("SF_JUNCTION", "-1"))
        if 0 <= sel < len(junctions):
            return junctions[sel]
        return min(junctions, key=lambda e: math.hypot(e[0] - rx, e[1] - ry))
    # 폴백: 가장 긴 통로 중앙
    big = max(boxes, key=lambda b: max(float(b[1][0] - b[0][0]), float(b[1][1] - b[0][1])))
    mn, mx = big
    return ((float(mn[0]) + float(mx[0])) / 2.0, (float(mn[1]) + float(mx[1])) / 2.0)


def find_corridor_endpoints(stage, barn_root, inset=1.2, merge=2.5):
    """Sand 통로들의 **양 끝점**(긴 축 양극단, 안쪽으로 inset)을 모아 가까운 것끼리 병합.
    → 순찰 웨이포인트(통로 끝 전부 + 교차점). 반환 [(x,y), ...] (월드)."""
    root = stage.GetPrimAtPath(barn_root)
    if not root or not root.IsValid():
        return []
    bbc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    pts = []
    for p in Usd.PrimRange(root):
        nm = p.GetName().lower()
        if p.GetTypeName() == "Mesh" and "sand" in nm and "corridor" in nm:
            r = bbc.ComputeWorldBound(p).ComputeAlignedRange()
            mn, mx = r.GetMin(), r.GetMax()
            cx, cy = (float(mn[0]) + float(mx[0])) / 2.0, (float(mn[1]) + float(mx[1])) / 2.0
            lx, ly = float(mx[0] - mn[0]), float(mx[1] - mn[1])
            if lx >= ly:
                pts.append((float(mn[0]) + inset, cy)); pts.append((float(mx[0]) - inset, cy))
            else:
                pts.append((cx, float(mn[1]) + inset)); pts.append((cx, float(mx[1]) - inset))
    merged = []   # (x, y, n)
    for x, y in pts:
        for i, (mxp, myp, n) in enumerate(merged):
            if math.hypot(x - mxp, y - myp) < merge:
                merged[i] = ((mxp * n + x) / (n + 1), (myp * n + y) / (n + 1), n + 1)
                break
        else:
            merged.append((x, y, 1))
    return [(x, y) for x, y, _ in merged]


def build_barn_occupancy(stage, barn_root, sx, sy, res=0.05, margin=1.5):
    """축사 콜라이더(펜스+투명 가상벽)와 바닥을 **점유격자**로 래스터화.
    벽=100(점유), 바닥=0(자유), 그 외=-1(미지). 시작점(sx,sy) 상대 origin.
    → 라이다 사각/투명벽 무관하게 '실제 알려진 맵'을 Nav2 에 제공."""
    root = stage.GetPrimAtPath(barn_root)
    if not root or not root.IsValid():
        return None
    bbc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    wall_kw = ("barrier", "escape", "fence", "wall", "rail")
    geom = ("Mesh", "Cube", "Cylinder", "Capsule", "Cone", "Sphere")
    walls, floors = [], []
    for p in Usd.PrimRange(root):
        if p.GetTypeName() not in geom:
            continue
        path = str(p.GetPath()).lower()
        nm = p.GetName().lower()
        r = bbc.ComputeWorldBound(p).ComputeAlignedRange()
        mn, mx = r.GetMin(), r.GetMax()
        bb = (float(mn[0]), float(mn[1]), float(mx[0]), float(mx[1]))
        if any(k in path for k in wall_kw):
            walls.append(bb)
        elif ("sand" in nm and "corridor" in nm) or "bedding" in nm or nm == "ground_mesh" or "floor" in nm:
            floors.append(bb)
    allb = walls + floors
    if not allb:
        return None
    xmin = min(b[0] for b in allb) - margin
    ymin = min(b[1] for b in allb) - margin
    xmax = max(b[2] for b in allb) + margin
    ymax = max(b[3] for b in allb) + margin
    W = int((xmax - xmin) / res) + 1
    H = int((ymax - ymin) / res) + 1
    grid = np.full((H, W), -1, dtype=np.int8)

    def _fill(bb, val):
        i0 = max(0, int((bb[0] - xmin) / res)); i1 = min(W, int((bb[2] - xmin) / res) + 1)
        j0 = max(0, int((bb[1] - ymin) / res)); j1 = min(H, int((bb[3] - ymin) / res) + 1)
        grid[j0:j1, i0:i1] = val

    for fb in floors:
        _fill(fb, 0)       # 자유(바닥)
    for wb in walls:
        _fill(wb, 100)     # 점유(벽 — 바닥 위에 덮어씀, 투명벽 포함)
    return grid, res, W, H, (xmin - sx, ymin - sy)


def bind_cow_texture(stage, cow_root, tex_path):
    """소 메시에 T_Cow_B 텍스처 바인딩. UsdUVTexture 는 **st(UV) 리더 연결 필수**
    (없으면 UV 없이 한 점만 샘플 → 평면 갈색). 메시 UV primvar 이름은 보통 'st'.
    """
    mat = UsdShade.Material.Define(stage, f"{cow_root}/CowMat")
    sh = UsdShade.Shader.Define(stage, f"{cow_root}/CowMat/Surface")
    sh.CreateIdAttr("UsdPreviewSurface")
    str_ = UsdShade.Shader.Define(stage, f"{cow_root}/CowMat/stReader")
    str_.CreateIdAttr("UsdPrimvarReader_float2")
    str_.CreateInput("varname", Sdf.ValueTypeNames.Token).Set(
        os.environ.get("SF_COW_UV", "st"))
    str_.CreateOutput("result", Sdf.ValueTypeNames.Float2)
    tx = UsdShade.Shader.Define(stage, f"{cow_root}/CowMat/Tex")
    tx.CreateIdAttr("UsdUVTexture")
    tx.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(tex_path)
    tx.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set("sRGB")
    tx.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(
        str_.ConnectableAPI(), "result")
    tx.CreateInput("wrapS", Sdf.ValueTypeNames.Token).Set("repeat")
    tx.CreateInput("wrapT", Sdf.ValueTypeNames.Token).Set("repeat")
    tx.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(
        tx.ConnectableAPI(), "rgb")
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.9)
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    n = 0
    for p in Usd.PrimRange(stage.GetPrimAtPath(cow_root)):
        if p.GetTypeName() == "Mesh":
            UsdShade.MaterialBindingAPI(p).Bind(
                mat, bindingStrength=UsdShade.Tokens.strongerThanDescendants)
            n += 1
    return n


def spawn_cow(stage, x, y, z, yaw, tex_path, prim="/World/Cow"):
    add_reference_to_stage(usd_path=COW_USD, prim_path=prim)
    # scenario1 검증 비균등 스케일(=실제 소 크기). SF_COW_SCALE 는 추가 배율.
    _sv = [2.37873, 1.69315, 1.69315]
    _m = float(os.environ.get("SF_COW_SCALE", "1.0"))
    xf = UsdGeom.Xformable(stage.GetPrimAtPath(prim))
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(x, y, z))
    xf.AddRotateXYZOp().Set(Gf.Vec3f(0.0, 0.0, math.degrees(yaw)))
    xf.AddScaleOp().Set(Gf.Vec3f(_sv[0] * _m, _sv[1] * _m, _sv[2] * _m))
    scale = _m
    simulation_app.update()
    # 실제 크기(월드 bbox) 측정 — 너무 작/크면 SF_COW_SCALE 로 조정
    try:
        bb = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                               [UsdGeom.Tokens.default_]).ComputeWorldBound(
            stage.GetPrimAtPath(prim)).ComputeAlignedRange()
        sz = bb.GetMax() - bb.GetMin()
        print(f"  📏 소 크기(L×W×H): {sz[0]:.2f}×{sz[1]:.2f}×{sz[2]:.2f} m (scale={scale})")
    except Exception:
        pass
    nb = 0
    if os.path.exists(tex_path):
        try:
            nb = bind_cow_texture(stage, prim, tex_path)
        except Exception as e:
            print(f"  ⚠️ 소 텍스처: {e}")
    print(f"  ✅ 소 스폰 {prim} @ world({x:.2f},{y:.2f}) yaw={math.degrees(yaw):.0f}° (텍스처 메시 {nb})")


# ─────────────────────────── RealSense 마운트 ────────────────────────
def mount_realsense(stage, robot_prim):
    _ee = None
    for _pref in ("arm0_link_fngr", "arm0_link_wr1", "arm0_link_wr0", "arm0_link_el1"):
        for _p in stage.Traverse():
            if _p.GetName() == _pref:
                _ee = str(_p.GetPath()); break
        if _ee:
            break
    _ee = _ee or robot_prim
    RS_MOUNT = f"{_ee}/RealSense"
    add_reference_to_stage(
        usd_path=f"{ISAAC_NUCLEUS_DIR}/Sensors/Intel/RealSense/rsd455.usd",
        prim_path=RS_MOUNT)
    _W = np.array([float(v) for v in os.environ.get("SF_RS_WORLD", "0.0,0.0,0.06").split(",")])
    _Mf = UsdGeom.Xformable(stage.GetPrimAtPath(_ee)).ComputeLocalToWorldTransform(
        Usd.TimeCode.Default())
    _RM = _Mf.ExtractRotationMatrix()
    _Rf = np.array([[_RM[i][j] for j in range(3)] for i in range(3)], float)
    _loc = _W @ _Rf.T
    _mx = UsdGeom.Xformable(stage.GetPrimAtPath(RS_MOUNT))
    _mx.ClearXformOpOrder()
    _mx.AddTranslateOp().Set(Gf.Vec3d(float(_loc[0]), float(_loc[1]), float(_loc[2])))
    _o = _mx.AddOrientOp()
    _o.Set(Gf.Quatf(1.0, Gf.Vec3f(0.0, 0.0, 0.0)))
    simulation_app.update()
    RS_COLOR = f"{RS_MOUNT}/{RS_COLOR_REL}"
    RS_DEPTH = f"{RS_MOUNT}/{RS_DEPTH_REL}"
    _M = UsdGeom.Camera(stage.GetPrimAtPath(RS_COLOR)).ComputeLocalToWorldTransform(
        Usd.TimeCode.Default())
    _R = _M.ExtractRotationMatrix()
    _f = np.array([-_R[2][0], -_R[2][1], -_R[2][2]], float)
    _u = np.array([_R[1][0], _R[1][1], _R[1][2]], float)
    _f /= (np.linalg.norm(_f) + 1e-9); _u /= (np.linalg.norm(_u) + 1e-9)
    _Mc = np.column_stack([_f, _u, np.cross(_f, _u)])
    _Mt = np.column_stack([[1., 0, 0], [0, 0, 1.], [0, -1., 0]])
    _Rm = _Mt @ _Mc.T
    _tr = _Rm[0, 0] + _Rm[1, 1] + _Rm[2, 2]
    if _tr > 0:
        _s = math.sqrt(_tr + 1.0) * 2
        _qw = 0.25 * _s; _qx = (_Rm[2, 1] - _Rm[1, 2]) / _s
        _qy = (_Rm[0, 2] - _Rm[2, 0]) / _s; _qz = (_Rm[1, 0] - _Rm[0, 1]) / _s
    elif _Rm[0, 0] > _Rm[1, 1] and _Rm[0, 0] > _Rm[2, 2]:
        _s = math.sqrt(1.0 + _Rm[0, 0] - _Rm[1, 1] - _Rm[2, 2]) * 2
        _qw = (_Rm[2, 1] - _Rm[1, 2]) / _s; _qx = 0.25 * _s
        _qy = (_Rm[0, 1] + _Rm[1, 0]) / _s; _qz = (_Rm[0, 2] + _Rm[2, 0]) / _s
    elif _Rm[1, 1] > _Rm[2, 2]:
        _s = math.sqrt(1.0 + _Rm[1, 1] - _Rm[0, 0] - _Rm[2, 2]) * 2
        _qw = (_Rm[0, 2] - _Rm[2, 0]) / _s; _qx = (_Rm[0, 1] + _Rm[1, 0]) / _s
        _qy = 0.25 * _s; _qz = (_Rm[1, 2] + _Rm[2, 1]) / _s
    else:
        _s = math.sqrt(1.0 + _Rm[2, 2] - _Rm[0, 0] - _Rm[1, 1]) * 2
        _qw = (_Rm[1, 0] - _Rm[0, 1]) / _s; _qx = (_Rm[0, 2] + _Rm[2, 0]) / _s
        _qy = (_Rm[1, 2] + _Rm[2, 1]) / _s; _qz = 0.25 * _s
    _o.Set(Gf.Quatf(float(_qw), Gf.Vec3f(float(_qx), float(_qy), float(_qz))))
    simulation_app.update()
    for _cp in (RS_COLOR, f"{RS_MOUNT}/{RS_SUB}/Camera_OmniVision_OV9782_Left",
                f"{RS_MOUNT}/{RS_SUB}/Camera_OmniVision_OV9782_Right"):
        _c = stage.GetPrimAtPath(_cp)
        if _c and _c.IsValid():
            UsdGeom.Camera(_c).GetClippingRangeAttr().Set(Gf.Vec2f(0.01, 1000.0))
    _dp = stage.GetPrimAtPath(RS_DEPTH)
    if _dp and _dp.IsValid():
        UsdGeom.Camera(_dp).GetClippingRangeAttr().Set(Gf.Vec2f(0.05, 10.0))
    print("  ✅ 손 RealSense 마운트+정렬")
    return RS_COLOR, RS_DEPTH, _ee


# ─────────────────────────── ROS 그래프 ──────────────────────────────
def build_ros_graph(rp_color, rp_depth, lidar_rp):
    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": GRAPH, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("Tick", "omni.graph.action.OnPlaybackTick"),
                ("SimTime", f"{NS_CORE}.IsaacReadSimulationTime"),
                ("ClockPub", f"{NS_ROS2}.ROS2PublishClock"),
                ("SubTwist", f"{NS_ROS2}.ROS2SubscribeTwist"),
                ("OdomPub", f"{NS_ROS2}.ROS2PublishOdometry"),
                ("TfOdom", f"{NS_ROS2}.ROS2PublishRawTransformTree"),
                ("TfLidar", f"{NS_ROS2}.ROS2PublishRawTransformTree"),
                ("TfCam", f"{NS_ROS2}.ROS2PublishRawTransformTree"),
                ("LidarPub", f"{NS_ROS2}.ROS2RtxLidarHelper"),
                ("RgbPub", f"{NS_ROS2}.ROS2CameraHelper"),
                ("DepthPub", f"{NS_ROS2}.ROS2CameraHelper"),
                ("InfoPub", f"{NS_ROS2}.ROS2CameraInfoHelper"),
            ],
            keys.SET_VALUES: [
                ("ClockPub.inputs:topicName", "/clock"),
                ("SubTwist.inputs:topicName", "/cmd_vel"),
                ("OdomPub.inputs:topicName", "/odom"),
                ("OdomPub.inputs:odomFrameId", "odom"),
                ("OdomPub.inputs:chassisFrameId", "base_link"),
                ("TfOdom.inputs:topicName", "/tf"),
                ("TfOdom.inputs:parentFrameId", "odom"),
                ("TfOdom.inputs:childFrameId", "base_link"),
                # base_link → lidar_frame (static)
                ("TfLidar.inputs:topicName", "/tf_static"),
                ("TfLidar.inputs:parentFrameId", "base_link"),
                ("TfLidar.inputs:childFrameId", FRAME_LIDAR),
                ("TfLidar.inputs:translation", [0.0, 0.0, LIDAR_MOUNT_Z]),
                ("TfLidar.inputs:staticPublisher", True),
                # base_link → spot_cam (동적, 손 카메라 FK) — 비전 꼬리 역투영용
                ("TfCam.inputs:topicName", "/tf"),
                ("TfCam.inputs:parentFrameId", "base_link"),
                ("TfCam.inputs:childFrameId", "spot_cam"),
                # RTX 라이다 → /scan
                ("LidarPub.inputs:renderProductPath", lidar_rp or ""),
                ("LidarPub.inputs:topicName", TOPIC_SCAN),
                ("LidarPub.inputs:frameId", FRAME_LIDAR),
                ("LidarPub.inputs:type", "laser_scan"),
                ("RgbPub.inputs:renderProductPath", rp_color),
                ("RgbPub.inputs:type", "rgb"),
                ("RgbPub.inputs:topicName", "/spot_cam/rgb"),
                ("RgbPub.inputs:frameId", "spot_cam"),
                ("DepthPub.inputs:renderProductPath", rp_depth),
                ("DepthPub.inputs:type", "depth"),
                ("DepthPub.inputs:topicName", "/spot_cam/depth"),
                ("DepthPub.inputs:frameId", "spot_cam"),
                ("InfoPub.inputs:renderProductPath", rp_color),
                ("InfoPub.inputs:topicName", "/spot_cam/camera_info"),
                ("InfoPub.inputs:frameId", "spot_cam"),
            ],
            keys.CONNECT: [
                ("Tick.outputs:tick", "ClockPub.inputs:execIn"),
                ("Tick.outputs:tick", "SubTwist.inputs:execIn"),
                ("Tick.outputs:tick", "OdomPub.inputs:execIn"),
                ("Tick.outputs:tick", "TfOdom.inputs:execIn"),
                ("Tick.outputs:tick", "TfLidar.inputs:execIn"),
                ("Tick.outputs:tick", "TfCam.inputs:execIn"),
                ("Tick.outputs:tick", "LidarPub.inputs:execIn"),
                ("Tick.outputs:tick", "RgbPub.inputs:execIn"),
                ("Tick.outputs:tick", "DepthPub.inputs:execIn"),
                ("Tick.outputs:tick", "InfoPub.inputs:execIn"),
                ("SimTime.outputs:simulationTime", "ClockPub.inputs:timeStamp"),
                ("SimTime.outputs:simulationTime", "OdomPub.inputs:timeStamp"),
                ("SimTime.outputs:simulationTime", "TfOdom.inputs:timeStamp"),
                ("SimTime.outputs:simulationTime", "TfCam.inputs:timeStamp"),
            ],
        },
    )
    A = og.Controller.attribute
    return {
        "sub_lin": A(f"{GRAPH}/SubTwist.outputs:linearVelocity"),
        "sub_ang": A(f"{GRAPH}/SubTwist.outputs:angularVelocity"),
        "od_pos": A(f"{GRAPH}/OdomPub.inputs:position"),
        "od_ori": A(f"{GRAPH}/OdomPub.inputs:orientation"),
        "tf_t": A(f"{GRAPH}/TfOdom.inputs:translation"),
        "tf_r": A(f"{GRAPH}/TfOdom.inputs:rotation"),
        "tfc_t": A(f"{GRAPH}/TfCam.inputs:translation"),
        "tfc_r": A(f"{GRAPH}/TfCam.inputs:rotation"),
    }


# ─────────────────────────── go-to-goal ──────────────────────────────
def goto_goal(robot, gx, gy, v_max=0.9, slow_r=1.2, k_yaw=1.6, w_max=1.2):
    pos = robot.data.root_pos_w[0]
    yaw = float(euler_xyz_from_quat(robot.data.root_quat_w[0].unsqueeze(0))[2][0])
    dx, dy = gx - float(pos[0]), gy - float(pos[1])
    dist = math.hypot(dx, dy)
    yaw_err = float(wrap_to_pi(torch.tensor(math.atan2(dy, dx) - yaw)))
    wz = max(-w_max, min(w_max, k_yaw * yaw_err))
    vx = v_max * max(0.0, math.cos(yaw_err)) * min(1.0, dist / slow_r)
    return vx, wz, dist, yaw


def detect_tail_world(model, rgb_annot, depth_annot, cam_K, cwp, cwq, device):
    """렌더 RGB+Depth(annotator) → YOLO tail_base(kpt5) → optical 3D 역투영 →
    카메라 월드포즈(cwp 위치, cwq optical 회전)로 world (x,y) 산출. 실패시 None.
    cow_tail_seek.py 의 ROS 파이프라인과 동일 로직을 프로세스 내부에서 수행."""
    try:
        rgb = np.asarray(rgb_annot.get_data())
        depth = np.asarray(depth_annot.get_data())
    except Exception:
        return None
    if rgb.ndim != 3 or rgb.size == 0 or depth.ndim < 2 or depth.size == 0:
        return None
    bgr = np.ascontiguousarray(rgb[:, :, 2::-1])   # RGBA→BGR (cv2 규약, cow_tail_seek 동일)
    r = model(bgr, conf=DET_CONF, verbose=False)[0]
    if SAVE_DET:                                    # 640 검출 출력(annotated) 홈에 저장
        try:
            import cv2
            cv2.imwrite(SAVE_DET_PATH, r.plot())    # bbox+키포인트 오버레이
        except Exception:
            pass
    if r.keypoints is None or r.boxes is None or len(r.boxes) == 0:
        return None
    kps = r.keypoints.data.cpu().numpy()           # (n,14,3)
    bi = int(np.argmax(r.boxes.conf.cpu().numpy()))   # 가장 신뢰 높은 소
    # 오검출/원거리 거름: 소 bbox 가 충분히 커야(=가까워야) 신뢰(근거리 검출은 6cm로 정확).
    _box = r.boxes.xyxy.cpu().numpy()[bi]
    if float(_box[3] - _box[1]) < MIN_BBOX_FRAC * rgb.shape[0]:
        return None
    tail = kps[bi, TAIL_KPT]                        # [x, y, conf]
    if float(tail[2]) < KPT_CONF:
        return None
    u, v = float(tail[0]), float(tail[1])
    dh, dw = depth.shape[:2]
    rh, rw = rgb.shape[:2]
    du, dv = int(u * dw / rw), int(v * dh / rh)
    if not (0 <= dv < dh and 0 <= du < dw):
        return None
    d = float(depth[dv, du])
    if not (np.isfinite(d) and 0.1 < d < 30.0):
        return None
    fx, fy, cx, cy = cam_K
    p_opt = torch.tensor([[(u - cx) * d / fx, (v - cy) * d / fy, d]],
                         device=device, dtype=torch.float32)   # optical: z전방,x우,y하
    p_world = (cwp + quat_apply(cwq, p_opt))[0]
    return float(p_world[0]), float(p_world[1])


def main():
    print("=" * 60)
    print(" 시나리오: 순찰 → 소 발견 → 후방(꼬리) 이동")
    print(f"  [env] numpy={np.__version__} torch={torch.__version__}")
    print("=" * 60)
    env_cfg = parse_env_cfg(a.task, device="cuda:0", num_envs=1)
    try:
        for _n in [n for n in vars(env_cfg.terminations) if not n.startswith("_")]:
            setattr(env_cfg.terminations, _n, None)
    except Exception:
        pass
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
    print(f"  ✅ 정책: {a.policy}")

    env = gym.make(a.task, cfg=env_cfg)
    obs, _ = env.reset()
    device = env.unwrapped.device
    stage = omni.usd.get_context().get_stage()
    robot = env.unwrapped.scene["robot"]
    robot_prim = robot.root_physx_view.prim_paths[0]

    policy = torch.jit.load(a.policy, map_location=device).eval()
    cmd_term = env.unwrapped.command_manager.get_term("base_velocity")
    try:
        cmd_term.cfg.resampling_time_range = (1.0e9, 1.0e9)
        cmd_term.cfg.heading_command = False
        cmd_term.vel_command_b[:] = 0.0
    except Exception:
        pass

    # ── 팔 stow(접기) 준비 — 즉시 접으면 토크로 뒤집힘. **맵 구성 후 천천히** 접는다.
    #  평면정책이 무팔(spot_flat)이라 팔 뻗으면 CoM 쏠림 → stow 로 중앙화(안정).
    _jn = list(robot.joint_names)
    # ── 팔 무게 가볍게(massless 트릭) — 무팔 강건 보행정책 그대로 안정 보행 ──
    #  SF_ARM_LIGHT_KG>0 시 팔 링크 질량·관성을 그 값으로 낮춤(CoM 외란 제거).
    _arm_light = float(os.environ.get("SF_ARM_LIGHT_KG", "0"))
    if _arm_light > 0:
        import sys as _syslm
        _root_lm = os.path.dirname(_ASSETS)
        if _root_lm not in _syslm.path:
            _syslm.path.insert(0, _root_lm)
        try:
            from arm_mass import apply_light_arm
            _nlm = apply_light_arm(robot, _arm_light)
            print(f"  ✅ 팔 경량화 {_nlm}개 링크 → {_arm_light}kg "
                  f"(CoM 외란↓ — 무팔 강건정책 안정 보행)")
        except Exception as _elm:
            print(f"  ⚠️ 팔 경량화 셋업 실패(무시): {_elm}")
    #  등쪽(背) 수납: 어깨(sh1)를 한계 가까이(-3.05) 위·뒤로 돌리고 팔꿈치(el0)를
    #  한계 가까이(3.05) 완전히 접어 → 팔(전완)이 등 위에 눕는다. CoM 최저·중앙.
    _stow = {"arm0_sh0": float(os.environ.get("SF_STOW_SH0", "0.0")),
             "arm0_sh1": float(os.environ.get("SF_STOW_SH1", "-3.05")),
             "arm0_el0": float(os.environ.get("SF_STOW_EL0", "3.05")),
             "arm0_el1": 0.0, "arm0_wr0": 0.0, "arm0_wr1": 0.0}
    arm_ids = [i for i, n in enumerate(_jn) if n in _stow]
    arm_default = arm_stow_vec = arm_open_vec = None
    # 팔 완전 펼침(다 펼침) 자세 — 어깨·팔꿈치·손목까지 곧게 수평 전방으로 쭉.
    _open = {"arm0_sh0": float(os.environ.get("SF_OPEN_SH0", "0.0")),
             "arm0_sh1": float(os.environ.get("SF_OPEN_SH1", "-0.3")),
             "arm0_el0": float(os.environ.get("SF_OPEN_EL0", "0.0")),
             "arm0_el1": 0.0, "arm0_wr0": float(os.environ.get("SF_OPEN_WR0", "0.6")),
             "arm0_wr1": 0.0}
    if arm_ids:
        arm_default = robot.data.joint_pos[0, arm_ids].clone()
        arm_stow_vec = torch.tensor([_stow[_jn[i]] for i in arm_ids],
                                    device=device, dtype=torch.float32)
        arm_open_vec = torch.tensor([_open[_jn[i]] for i in arm_ids],
                                    device=device, dtype=torch.float32)
        # 실제 관절 한계 출력(수납 자세 방향/범위 검증용)
        _lim = None
        for _a in ("joint_pos_limits", "soft_joint_pos_limits", "default_joint_pos_limits"):
            if hasattr(robot.data, _a):
                _lim = getattr(robot.data, _a)[0]; break
        for i in arm_ids:
            _df = float(arm_default[arm_ids.index(i)])
            if _lim is not None:
                lo, hi = float(_lim[i, 0]), float(_lim[i, 1])
                print(f"     [한계] {_jn[i]:10s} [{lo:+.2f},{hi:+.2f}] 기본={_df:+.2f} → 수납={_stow[_jn[i]]:+.2f}")
            else:
                print(f"     {_jn[i]:10s} 기본={_df:+.2f} → 수납={_stow[_jn[i]]:+.2f}")
        print(f"  ✅ 팔 {len(arm_ids)}관절 — 시작 정지구간에 제자리 수납")
    # [수납/정지 보류] 팔정책 재학습 후 적용 — 시작 정지(HOLD)·제자리 수납·다리벌림 준비 비활성.
    #  (재학습 후 아래 블록 복원)
    #  ctrl_dt = float(getattr(env.unwrapped, "step_dt", 0.02)) or 0.02
    #  STOP_SECS = float(os.environ.get("SF_STOP_SECS", "20"))    # 처음 N초 정지
    #  HOLD_STEPS = int(STOP_SECS / ctrl_dt)
    #  FOLD_START = int(os.environ.get("SF_FOLD_START", str(int(HOLD_STEPS * 0.20))))
    #  FOLD_DUR = int(os.environ.get("SF_FOLD_DUR", str(max(60, HOLD_STEPS - FOLD_START - int(0.10*HOLD_STEPS)))))
    #  SPREAD = float(os.environ.get("SF_LEG_SPREAD", "0.25")); _ascale = 0.2
    #  hx_act = []
    #  try:
    #      _am = env.unwrapped.action_manager
    #      _at = _am.get_term("joint_pos") if hasattr(_am, "get_term") else list(_am._terms.values())[0]
    #      _aids = list(getattr(_at, "_joint_ids", getattr(_at, "joint_ids", [])))
    #      for _ai, _ji in enumerate(_aids):
    #          _nm = _jn[_ji]
    #          if _nm.endswith("_hx"):
    #              hx_act.append((_ai, (1.0 if _nm[1]=="l" else -1.0) * SPREAD / _ascale))
    #  except Exception: hx_act = []
    # 순수 시작 정지 — 수납/다리벌림 없이, SLAM·Nav2 가 뜰 때까지 제자리(초기 배회 방지).
    _cdt = float(getattr(env.unwrapped, "step_dt", 0.02)) or 0.02
    START_STOP_SECS = float(os.environ.get("SF_START_STOP_SECS", "90"))   # 시작 안정화(짧게)
    START_STOP_STEPS = int(START_STOP_SECS / _cdt)
    print(f"  ⏸ 시작 정지 {START_STOP_SECS:.0f}s(={START_STOP_STEPS}스텝) — Nav2 활성까지 제자리(팔 펼침 유지)")
    # 정지 중 다리 자세 조정 — 정책 액션에 hip_x 벌림 바이어스만 더함(균형은 정책이 유지).
    LEG_TUNE = float(os.environ.get("SF_LEG_TUNE", "0.6"))   # 정지중 hip_x 외전 바이어스(액션단위)
    _hx_idx = []
    try:
        _am = env.unwrapped.action_manager
        _at = _am.get_term("joint_pos") if hasattr(_am, "get_term") else list(_am._terms.values())[0]
        _aids = list(getattr(_at, "_joint_ids", getattr(_at, "joint_ids", [])))
        for _ai, _ji in enumerate(_aids):
            if _jn[_ji].endswith("_hx"):
                _hx_idx.append((_ai, 1.0 if _jn[_ji][1] == "l" else -1.0))
        print(f"  ✅ 정지중 다리조정: hip_x {len(_hx_idx)}개 (벌림 바이어스 {LEG_TUNE})")
    except Exception as _e:
        print(f"  ⚠️ 다리조정 준비: {_e}")

    # 안착(워밍업) — 팔은 기본자세 유지(접지 않음)
    for _ in range(60):
        with torch.inference_mode():
            obs, _, _, _, _ = env.step(policy(obs["policy"] if isinstance(obs, dict) else obs))
    _rs = robot.data.root_state_w[0]
    SX, SY = float(_rs[0]), float(_rs[1])
    _fz = float(robot.data.body_pos_w[0, :, 2].min())
    FLOOR_Z = _fz - 0.20
    # 축사/소 z 내림(정렬). 축사 바닥은 로봇 발과 정렬→기본 0, 필요시 SF_BARN_DZ 로 내림.
    BARN_DZ = float(os.environ.get("SF_BARN_DZ", "0.0"))    # 축사 전체 내림
    COW_DZ = float(os.environ.get("SF_COW_DZ", "-0.15"))    # 소 내림(떠있음 보정)
    BARN_Z = FLOOR_Z + BARN_DZ
    COW_Z = FLOOR_Z + BARN_DZ + COW_DZ
    print(f"  ✅ 안착: 시작({SX:.2f},{SY:.2f}) 바닥z={FLOOR_Z:.3f} | 축사z={BARN_Z:.3f} 소z={COW_Z:.3f}")

    # 축사
    if os.path.exists(ENV_USD):
        add_reference_to_stage(usd_path=ENV_USD, prim_path="/World/Barn")
        _bx = UsdGeom.Xformable(stage.GetPrimAtPath("/World/Barn"))
        _bx.ClearXformOpOrder()
        _bx.AddTranslateOp().Set(Gf.Vec3d(SX, SY, BARN_Z))
        simulation_app.update()
        # 시작점 = 통로 끝(중앙 X): 선택한 통로 끝이 로봇 스폰(SX,SY)에 오도록 축사 이동(고정 버전).
        if os.environ.get("SF_START_AT_END", "1") == "1":
            _ends = find_corridor_endpoints(stage, "/World/Barn")
            if _ends:
                _ends.sort(key=lambda e: -math.hypot(e[0] - SX, e[1] - SY))   # 먼 끝 우선
                _sel = int(os.environ.get("SF_START_END_IDX", "0")) % len(_ends)
                _ex, _ey = _ends[_sel]
                _dx, _dy = SX - _ex, SY - _ey                                  # 그 끝 → 로봇 이동량
                _bx.ClearXformOpOrder()
                _bx.AddTranslateOp().Set(Gf.Vec3d(SX + _dx, SY + _dy, BARN_Z))
                simulation_app.update()
                print(f"  ✅ 시작점=통로 끝[{_sel}] — 축사 {_dx:+.1f},{_dy:+.1f} 이동(로봇이 통로 끝에서 출발)")
        if os.environ.get("SF_ROUGH", "0") != "1":
            apply_zone_friction(stage, "/World/Barn", PEN_FRICTION, CORRIDOR_FRICTION)
        apply_wall_collision(stage, "/World/Barn")
    for _gp in ("/World/ground", "/World/GroundPlane", "/World/defaultGroundPlane"):
        _g = stage.GetPrimAtPath(_gp)
        if _g and _g.IsValid():
            try:
                UsdGeom.Imageable(_g).MakeInvisible()
            except Exception:
                pass

    # 소(시작 기준 상대 → 월드)
    # 통로 끝(로봇 반대 방향)에 소 배치 — 열린 직진 경로. 소는 그쪽(기둥)을 보게.
    _ce = find_corridor_end(stage, "/World/Barn", SX, SY) \
        if os.environ.get("SF_COW_CORRIDOR", "1") == "1" else None
    if _ce:
        COW_X, COW_Y = _ce
        # 소 모델 정면축=+y → +90° 보정으로 머리=로봇 반대쪽, 꼬리=로봇쪽(반대로 봄)
        _off = math.radians(float(os.environ.get("SF_COW_YAW_OFF", "90")))
        cow_yaw = math.atan2(COW_Y - SY, COW_X - SX) + _off
        print(f"  ✅ 통로 끝(로봇 반대)에 소: ({COW_X:.2f},{COW_Y:.2f}) facing {math.degrees(cow_yaw):.0f}°")
    else:
        COW_X, COW_Y, cow_yaw = SX + COW_DX, SY + COW_DY, COW_YAW
    spawn_cow(stage, COW_X, COW_Y, COW_Z, cow_yaw, COW_TEX)

    # ── 소 bbox로 꼬리 끝 계산 → "꼬리 1.5m 뒤" 목표(계산 기반) ──
    BEHIND = float(os.environ.get("SF_BEHIND_TAIL", "1.5"))
    TAIL_SIGN = float(os.environ.get("SF_TAIL_SIGN", "1"))    # 소 180° 회전에 맞춰 꼬리쪽(+fwd)
    try:
        _bb = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_]
                                ).ComputeWorldBound(
            stage.GetPrimAtPath("/World/Cow")).ComputeAlignedRange()
        _mn, _mx = _bb.GetMin(), _bb.GetMax()
        _lx, _ly = float(_mx[0] - _mn[0]), float(_mx[1] - _mn[1])
    except Exception:
        _lx, _ly = 1.0, 3.2
    # 몸 길이축 = 더 긴 수평축. 그 방향을 forward, 꼬리는 TAIL_SIGN*forward 끝.
    if _lx >= _ly:
        cow_fwd = (1.0, 0.0); half_fwd = _lx / 2.0
    else:
        cow_fwd = (0.0, 1.0); half_fwd = _ly / 2.0
    # ── 시나리오용 추가 소(간격 배치) — cow0 옆으로 COW_SPACING 씩. 데모=1마리면 스킵 ──
    if COW_COUNT > 1:
        _perp = (-cow_fwd[1], cow_fwd[0])    # 몸 길이축 수직 = 옆줄 방향
        for _i in range(1, COW_COUNT):
            _cx = COW_X + _perp[0] * _i * COW_SPACING
            _cy = COW_Y + _perp[1] * _i * COW_SPACING
            spawn_cow(stage, _cx, _cy, COW_Z, cow_yaw, COW_TEX, prim=f"/World/Cow_{_i:02d}")
        print(f"  🐄 시나리오 소 {COW_COUNT}마리 배치(간격 {COW_SPACING}m) — cow0=검사기준")

    TAIL_X = COW_X + TAIL_SIGN * cow_fwd[0] * half_fwd          # 꼬리 끝
    TAIL_Y = COW_Y + TAIL_SIGN * cow_fwd[1] * half_fwd
    REAR_X = TAIL_X + TAIL_SIGN * cow_fwd[0] * BEHIND           # 꼬리 1.5m 뒤
    REAR_Y = TAIL_Y + TAIL_SIGN * cow_fwd[1] * BEHIND
    print(f"  📐 소중심({COW_X:.2f},{COW_Y:.2f}) 반길이={half_fwd:.2f} "
          f"→ 꼬리({TAIL_X:.2f},{TAIL_Y:.2f}) → 목표(꼬리{BEHIND}m뒤)({REAR_X:.2f},{REAR_Y:.2f})")
    # 시작점(=map/odom 원점) 상대 목표 저장 → goal-sender(Nav2 NavigateToPose)가 사용.
    #  비전모드(SF_VISION_TAIL=1)에선 정답좌표를 쓰지 않음 → 루프의 in-process YOLO 검출이
    #  안정 추정 후 이 파일을 갱신한다. 이전 실행 잔여 파일은 stale 방지로 삭제.
    if VISION_TAIL:
        try:
            if os.path.exists("/tmp/scenario_goal.json"):
                os.remove("/tmp/scenario_goal.json")
        except Exception:
            pass
        print("  🎯 비전모드: 시작 GT목표 비활성 — in-Isaac YOLO 검출이 목표 생성")
    else:
        try:
            import json
            with open("/tmp/scenario_goal.json", "w") as _gf:
                json.dump({"cow_rear": [REAR_X - SX, REAR_Y - SY],
                           "cow": [COW_X - SX, COW_Y - SY],
                           "tail": [TAIL_X - SX, TAIL_Y - SY]}, _gf)
            print(f"  ✅ 목표저장 /tmp/scenario_goal.json 후방(상대)="
                  f"({REAR_X - SX:.2f},{REAR_Y - SY:.2f})")
        except Exception as e:
            print(f"  ⚠️ 목표저장: {e}")

    # ── 순찰 웨이포인트: 통로 끝 전부 추출 → 시작상대 저장(patrol.py가 순서대로 방문) ──
    try:
        import json
        _wp = find_corridor_endpoints(stage, "/World/Barn")
        _wp.sort(key=lambda e: math.hypot(e[0] - SX, e[1] - SY))   # 시작점 가까운 순
        with open("/tmp/patrol_waypoints.json", "w") as _pf:
            json.dump({"waypoints": [[x - SX, y - SY] for x, y in _wp]}, _pf)
        print(f"  ✅ 순찰 웨이포인트 {len(_wp)}개(통로 끝): "
              f"{[(round(x - SX, 1), round(y - SY, 1)) for x, y in _wp]}")
    except Exception as e:
        print(f"  ⚠️ 순찰 웨이포인트: {e}")

    # ── 알려진 맵: 콜라이더(펜스+투명 가상벽) → 점유격자 저장(barn_map_server 가 /map 발행) ──
    try:
        import json
        _bm = build_barn_occupancy(stage, "/World/Barn", SX, SY)
        if _bm:
            _grid, _res, _W, _H, _org = _bm
            np.save("/tmp/barn_map.npy", _grid)
            with open("/tmp/barn_map.json", "w") as _mf:
                json.dump({"resolution": _res, "width": _W, "height": _H,
                           "origin": [_org[0], _org[1]]}, _mf)
            _nocc = int((_grid == 100).sum())
            print(f"  ✅ 알려진 맵 생성: {_W}×{_H}@{_res}m 점유셀{_nocc}(투명벽 포함) → /tmp/barn_map.*")
    except Exception as e:
        print(f"  ⚠️ 알려진 맵 생성: {e}")

    # 카메라 + ROS
    RS_COLOR, RS_DEPTH, _EE = mount_realsense(stage, robot_prim)
    # 카메라→손링크(ee) 고정 오프셋(rigid) — base_link→spot_cam 동적 TF 발행용(비전 꼬리 역투영)
    cam_in_ee_pos = cam_in_ee_quat = ee_idx = None
    _Q_RX_PI = torch.tensor([[0.0, 1.0, 0.0, 0.0]], device=device)  # Rx(180°) wxyz: USD카메라→ROS optical
    try:
        _ee_name = _EE.rstrip("/").split("/")[-1]
        ee_idx = list(robot.body_names).index(_ee_name)
        _Mcam = UsdGeom.Camera(stage.GetPrimAtPath(RS_COLOR)).ComputeLocalToWorldTransform(
            Usd.TimeCode.Default())
        _ct = _Mcam.ExtractTranslation()
        _cq = _Mcam.ExtractRotationQuat()
        _cqi = _cq.GetImaginary()
        _camp = torch.tensor([[float(_ct[0]), float(_ct[1]), float(_ct[2])]], device=device)
        _camq = torch.tensor([[float(_cq.GetReal()), float(_cqi[0]), float(_cqi[1]), float(_cqi[2])]],
                             device=device)  # wxyz
        _eep = robot.data.body_pos_w[0, ee_idx].unsqueeze(0)
        _eeq = robot.data.body_quat_w[0, ee_idx].unsqueeze(0)
        cam_in_ee_pos = quat_apply_inverse(_eeq, _camp - _eep)
        cam_in_ee_quat = quat_mul(quat_inv(_eeq), _camq)
        print(f"  ✅ 카메라 TF 준비: base_link→spot_cam 동적(ee={_ee_name}, idx={ee_idx})")
    except Exception as _e:
        print(f"  ⚠️ 카메라 TF 준비 실패: {_e}")
    rp_color = rep.create.render_product(RS_COLOR, RES)
    rp_depth = rep.create.render_product(RS_DEPTH, RES)
    rpc = rp_color.path if hasattr(rp_color, "path") else str(rp_color)
    rpd = rp_depth.path if hasattr(rp_depth, "path") else str(rp_depth)
    # in-Isaac 비전(YOLO+annotator) 준비는 ROS 그래프 빌드·timeline.play() 이후로 미룬다
    #  (렌더 프레임이 있어야 annotator 버퍼가 유효 + 그래프 빌드 간섭 방지 — 둘 다 조기 attach 문제)
    yolo_model = rgb_annot = depth_annot = cam_K = None

    # ── RTX 라이다(베이스 마운트) → /scan ──
    lidar_mount = f"{robot_prim}/Lidar360"
    _lm = UsdGeom.Xform.Define(stage, lidar_mount)
    _lx = UsdGeom.Xformable(_lm.GetPrim())
    _lx.ClearXformOpOrder()
    _lx.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, LIDAR_MOUNT_Z))
    lidar_prim = None
    try:
        _, _ld = omni.kit.commands.execute(
            "IsaacSensorCreateRtxLidar", path="/lidar_sensor",
            parent=lidar_mount, config=LIDAR_CONFIG)
        lidar_prim = f"{lidar_mount}/lidar_sensor"
        print(f"  ✅ RTX Lidar: {lidar_prim} ({LIDAR_CONFIG})")
    except Exception as e:
        print(f"  ❌ Lidar 생성: {e}")
    simulation_app.update()
    lidar_rp = None
    if lidar_prim:
        try:
            _lr = rep.create.render_product(lidar_prim, [1, 1])
            lidar_rp = _lr.path if hasattr(_lr, "path") else str(_lr)
        except Exception as e:
            print(f"  ❌ Lidar 렌더프로덕트: {e}")

    attrs = build_ros_graph(rpc, rpd, lidar_rp)
    omni.timeline.get_timeline_interface().play()
    simulation_app.update()
    print("  ✅ ROS: /clock /odom /tf /scan /spot_cam/{rgb,depth,camera_info}")

    # ── in-Isaac 비전: 검출 전용 render product + YOLO + intrinsics (play 이후) ──
    #  ROS용 render product 와 분리해 그래프 간섭을 막고, 렌더 프레임 확보 후 attach 한다.
    if VISION_TAIL:
        try:
            from ultralytics import YOLO
            yolo_model = YOLO(os.path.join(_ASSETS, "yolo", "best.pt"))
            _vis_rp = rep.create.render_product(RS_COLOR, RES)
            rgb_annot = rep.AnnotatorRegistry.get_annotator("rgb")
            rgb_annot.attach([_vis_rp])
            # depth 를 같은(컬러) render product 에 → RGB 픽셀과 정합, color intrinsics 일치
            depth_annot = rep.AnnotatorRegistry.get_annotator("distance_to_image_plane")
            depth_annot.attach([_vis_rp])
            for _ in range(5):
                simulation_app.update()   # 첫 프레임 렌더(annotator 버퍼 채움)
            _cam = UsdGeom.Camera(stage.GetPrimAtPath(RS_COLOR))
            _f = float(_cam.GetFocalLengthAttr().Get() or 0.0)
            _ha = float(_cam.GetHorizontalApertureAttr().Get() or 0.0)
            _Wpx, _Hpx = RES
            # Isaac 는 render product 를 정사각/임의 해상도로 렌더해도 픽셀은 정사각 →
            #  fy = fx (USD verticalAperture 는 센서 native 16:9 라 렌더와 불일치, 쓰면 안 됨)
            fx = (_Wpx * _f / _ha) if _ha > 0 else float(_Wpx)
            fy = fx
            cam_K = (fx, fy, _Wpx / 2.0, _Hpx / 2.0)
            print(f"  ✅ in-Isaac YOLO 준비: best.pt | K=fx{fx:.1f} fy{fy:.1f} "
                  f"cx{cam_K[2]:.1f} cy{cam_K[3]:.1f} @ {_Wpx}x{_Hpx}")
        except Exception as _e:
            print(f"  ⚠️ in-Isaac 비전 준비 실패 → 비활성: {_e}")
            yolo_model = None

    # ── 디버그 3인칭 시점 — 매번 뷰 조정 안 하게 축사 전체(시작~소)를 옆·위에서 고정 조망 ──
    if not _HEADLESS:
        _scv = None
        for _mod in ("isaacsim.core.utils.viewports", "omni.isaac.core.utils.viewports"):
            try:
                _scv = __import__(_mod, fromlist=["set_camera_view"]).set_camera_view; break
            except Exception:
                pass
        if _scv is not None:
            _cx, _cy = SX, (SY + COW_Y) / 2.0            # 시작점~소 중간
            _eye = [_cx + float(os.environ.get("SF_DBG_EYE_X", "8")),
                    _cy + float(os.environ.get("SF_DBG_EYE_Y", "1")),
                    FLOOR_Z + float(os.environ.get("SF_DBG_EYE_Z", "8"))]
            try:
                _scv(eye=_eye, target=[_cx, _cy, FLOOR_Z])
                print(f"  ✅ 디버그 3인칭 시점 고정: eye={[round(v,1) for v in _eye]} → 축사중심({_cx:.1f},{_cy:.1f})")
            except Exception as _e:
                print(f"  ⚠️ 3인칭 시점 설정: {_e}")

    # ── 제어: SLAM+Nav2 가 /cmd_vel 을 주고, RL 정책이 그대로 보행 ──
    #  경로계획·장애물회피는 Nav2(라이다 코스트맵), 맵·로컬은 SLAM, 보행은 RL 정책.
    #  소 후방 목표는 외부 goal-sender(NavigateToPose)가 전송. 여기선 넘어짐 복구만.
    print(f"\n[RL 브릿지] /cmd_vel(Nav2) → 정책 보행 | 소 후방 목표=({REAR_X:.2f},{REAR_Y:.2f})")
    print("  시스템 ROS2(도메인153): SLAM(spot_slam) + Nav2(nav2_params_slam) + 목표전송\n")

    step = 0
    rec_t = 0
    x0 = y0 = None
    state = "RUN"
    nav_active = False   # Nav2 가 실제 명령 보내기 전까지 제자리(대기 중 완전 정지)
    vis_tails = []       # in-Isaac 비전: 최근 꼬리 월드 추정 누적(중앙값 안정화)
    vis_goal_xy = None   # 마지막으로 파일에 쓴 후방 목표(world) — 갱신 임계 판정용
    pid_vx_i = pid_wz_i = 0.0      # cmd_vel 추종 PID 적분항
    pid_vx_pe = pid_wz_pe = 0.0    # cmd_vel 추종 PID 직전 오차(D항)

    # ── INSPECT(후방 도착→앉아서 검사) 셋업 — 실험적, SF_INSPECT_ENABLE=1 로만 ──
    inspector = None
    leg_ids = []
    sit_leg_vec = stand_leg_vec = insp_arm_vec = None
    if INSPECT_ENABLE and arm_ids:
        try:
            import sys as _sys
            _root = os.path.dirname(_ASSETS)
            if _root not in _sys.path:
                _sys.path.insert(0, _root)
            from inspect_sequence import InspectSequence
            from arm_poses import leg_indices, sit_leg_targets, pose_vector, inspect_udder_pose
            inspector = InspectSequence()
            leg_ids = leg_indices(_jn)
            _def = robot.data.default_joint_pos[0].detach().cpu().tolist()
            _sit = sit_leg_targets(_def, _jn)
            sit_leg_vec = torch.tensor([_sit[i] for i in leg_ids], device=device)
            stand_leg_vec = torch.tensor([_def[i] for i in leg_ids], device=device)
            insp_arm_vec = torch.tensor(
                pose_vector(inspect_udder_pose(), [_jn[i] for i in arm_ids]),
                device=device, dtype=arm_open_vec.dtype)
            print(f"  ✅ INSPECT(앉아서 검사) 활성 — 다리 {len(leg_ids)} 팔 {len(arm_ids)} "
                  f"트리거={INSPECT_TRIGGER}")
        except Exception as _e:
            print(f"  ⚠️ INSPECT 셋업 실패(무시): {_e}")
            inspector = None

    while simulation_app.is_running():
        pos = robot.data.root_pos_w[0]
        rx, ry, rz = float(pos[0]), float(pos[1]), float(pos[2])

        # 넘어짐 복구(안전망)
        if state != "RECOVERY" and rz < FALL_Z:
            print(f"  ⚠️ 넘어짐(z={rz:.2f}) → 복구")
            state, rec_t = "RECOVERY", 0
        if state == "RECOVERY":
            vx, wz = 0.0, 0.0
            rec_t += 1
            if rec_t == REC_FORCE:
                with torch.inference_mode():
                    _r = robot.data.root_state_w.clone()
                    _r[:, 2] = STAND_Z
                    _r[:, 3], _r[:, 4], _r[:, 5], _r[:, 6] = 1.0, 0.0, 0.0, 0.0
                    robot.write_root_pose_to_sim(_r[:, :7])
                    robot.write_root_velocity_to_sim(torch.zeros_like(_r[:, 7:13]))
                print("  🔧 강제 기립")
            if rz > FALL_Z + 0.15 and rec_t > 30:
                print("  ✅ 복구 완료")
                state = "RUN"
        # [수납 보류] 팔정책 재학습 후 적용 — 시작 정지(HOLD) 구간 비활성.
        #  (재학습 후) elif step < HOLD_STEPS: vx,wz=0  # 제자리 균형 잡고 제자리 수납
        else:
            # Nav2 /cmd_vel 추종 — 순찰·소후방접근 **모두 Nav2(+SLAM+RL)** 가 구동.
            #  (in-Isaac 검출은 목표파일만 갱신 → 외부 relay가 순찰→소후방 전환)
            lin = og.Controller.get(attrs["sub_lin"])
            ang = og.Controller.get(attrs["sub_ang"])
            vx = float(lin[0]) if lin is not None else 0.0
            wz = float(ang[2]) if ang is not None else 0.0
            # 실제 명령(/cmd_vel≠0) 올 때까지 제자리(시작 드리프트 방지). 명령 오면 주행.
            if not nav_active:
                if abs(vx) > 0.03 or abs(wz) > 0.03:
                    nav_active = True
                    print(f"  ▶ Nav2 명령 감지(step={step}) — 주행 시작")
                elif step >= START_STOP_STEPS:
                    nav_active = True
                    print(f"  ▶ 정지 최대시간 도달 — 주행 시작")
                else:
                    vx, wz = 0.0, 0.0

        # ── cmd_vel 추종 PID(하부 속도 폐루프) ──
        #  Nav2 목표속도(vx,wz)를 로봇 실측 base 속도로 추종하도록 정책 입력명령을 보정.
        #  (정책의 속도추종 오차·관성을 피드백으로 메움. 정지 구간엔 적분 리셋으로 windup 방지)
        if CMD_PID and nav_active:
            try:
                _vm = float(robot.data.root_lin_vel_b[0, 0])   # 실측 전진속도(body x)
                _wm = float(robot.data.root_ang_vel_b[0, 2])   # 실측 yaw rate
            except Exception:
                _vm = _wm = None
            if _vm is not None:
                if abs(vx) < 0.02 and abs(wz) < 0.02:
                    pid_vx_i = pid_wz_i = 0.0
                    pid_vx_pe = pid_wz_pe = 0.0
                else:
                    _ev, _ew = vx - _vm, wz - _wm
                    pid_vx_i = max(-PID_I_CLAMP, min(PID_I_CLAMP, pid_vx_i + _ev * _cdt))
                    pid_wz_i = max(-PID_I_CLAMP, min(PID_I_CLAMP, pid_wz_i + _ew * _cdt))
                    vx = vx + PID_KP * _ev + PID_KI * pid_vx_i + PID_KD * (_ev - pid_vx_pe) / _cdt
                    wz = wz + PID_KP_W * _ew + PID_KI_W * pid_wz_i
                    pid_vx_pe, pid_wz_pe = _ev, _ew
                    vx = max(-PID_VX_LIM, min(PID_VX_LIM, vx))
                    wz = max(-PID_WZ_LIM, min(PID_WZ_LIM, wz))
        # ── INSPECT: 후방 도착 → 앉아서 검사 (실험적, 기본 OFF) ──
        #  주의: 다리 sit target 은 위치제어로 덮어쓰지만, 아래 env.step(정책액션)이
        #  다리 target 을 다시 쓰므로 **정책이 sit 을 거스를 수 있다**. 완전한 앉기는
        #  정지구간 정책 leg-action 억제(또는 sit-capable 정책)가 필요 → in-sim 검증 대상.
        #  cmd 정지 + 팔 검사자세 + 촬영 트리거는 그대로 동작한다.
        _inspecting = False
        if inspector is not None:
            if not inspector.is_active() and os.path.exists(INSPECT_TRIGGER):
                inspector.start(step)
                try:
                    os.remove(INSPECT_TRIGGER)
                except OSError:
                    pass
                print(f"  ★ INSPECT 시작(step={step}) — 정지→앉기→검사")
            _ins = inspector.update(step)
            _inspecting = _ins.active
            if _ins.active:
                vx, wz = 0.0, 0.0                       # 검사 중 주행 금지
                if leg_ids:
                    _legt = stand_leg_vec * (1.0 - _ins.leg_t) + sit_leg_vec * _ins.leg_t
                    robot.set_joint_position_target(_legt.unsqueeze(0), joint_ids=leg_ids)
                _armt = arm_open_vec * (1.0 - _ins.arm_t) + insp_arm_vec * _ins.arm_t
                robot.set_joint_position_target(_armt.unsqueeze(0), joint_ids=arm_ids)
                if _ins.capture:
                    # RGB-D 저장 + 유방염 열점(hotspot) 검출 → 클라우드 전송용 레코드
                    try:
                        import sys as _sys2, json as _json2, cv2 as _cv2
                        _root2 = os.path.dirname(_ASSETS)
                        for _pp in (_root2, os.path.join(_root2, "wip")):
                            if _pp not in _sys2.path:
                                _sys2.path.insert(0, _pp)
                        from inspection_capture import capture_paths, build_record
                        _od = os.environ.get("SF_INSPECT_OUT", "/tmp/inspect")
                        os.makedirs(_od, exist_ok=True)
                        _rgb = np.asarray(rgb_annot.get_data())[..., :3]
                        _dep = np.asarray(depth_annot.get_data())
                        _paths = capture_paths(_od, 0, step)
                        _cv2.imwrite(_paths["rgb"], _cv2.cvtColor(_rgb, _cv2.COLOR_RGB2BGR))
                        np.save(_paths["depth"], _dep)
                        _hot = None
                        try:    # 가짜 열화상(RGB Emission) → hotspot bbox (룰 기반, 학습 불요)
                            from thermal_processor import apply_fake_thermal, get_hotspot_bbox
                            _hot = get_hotspot_bbox(apply_fake_thermal(_rgb))
                        except Exception:
                            pass
                        _rec = build_record(0, (COW_X - SX, COW_Y - SY),
                                            _paths["rgb"], _paths["depth"], hotspot=_hot)
                        with open(_paths["meta"], "w") as _mf:
                            _json2.dump(_rec, _mf)
                        print(f"  📸 [검사] 촬영저장 {_paths['rgb']} "
                              f"hotspot={'있음' if _hot else '없음'} → 클라우드 대기")
                    except Exception as _e:
                        print(f"  ⚠️ 검사 촬영 저장 실패(무시): {_e}")
            if _ins.done:
                inspector.reset()
                print("  ✅ 검사 완료 — 일어섬, 순찰 재개")

        cmd_term.vel_command_b[:, 0] = vx
        cmd_term.vel_command_b[:, 1] = 0.0
        cmd_term.vel_command_b[:, 2] = wz
        # 팔: 펼침(extended) 자세로 부드럽게 전환 후 유지(손카메라가 소를 향함, 낮아 안정).
        if arm_ids and not _inspecting:
            _od = 100  # ~2s lerp(급변 방지)
            if step < _od:
                _t = step / _od
                _atgt = arm_default * (1.0 - _t) + arm_open_vec * _t
            else:
                _atgt = arm_open_vec
            robot.set_joint_position_target(_atgt.unsqueeze(0), joint_ids=arm_ids)
        # [수납 보류] 팔정책 재학습 후 적용 — 무게중심이 머리 위로 쏠려 뒤집힘.
        #  재학습(팔 관측 포함 SpotArm 정책) 후 아래 2단계 수납 + 다리벌림 고정액션을 켠다.
        #  if arm_ids:
        #      if step < FOLD_START: _atgt = arm_default
        #      else:  # 2단계: 팔꿈치 먼저 접고(_el) → 어깨를 등으로(_sh)
        #          _t = min(1.0, (step - FOLD_START) / max(1, FOLD_DUR))
        #          s_el = min(1.0, _t / 0.5); s_sh = max(0.0, min(1.0, (_t - 0.45) / 0.55))
        #          _atgt = arm_default.clone()
        #          for _k, _i in enumerate(arm_ids):
        #              _prog = s_sh if _jn[_i] == "arm0_sh1" else s_el
        #              _atgt[_k] = arm_default[_k]*(1.0-_prog) + arm_stow_vec[_k]*_prog
        #      robot.set_joint_position_target(_atgt.unsqueeze(0), joint_ids=arm_ids)
        with torch.inference_mode():
            _pa = policy(obs["policy"] if isinstance(obs, dict) else obs)
            # [수납 보류] 정지구간 다리벌림 고정액션도 재학습 후 활성:
            #  if state != "RECOVERY" and step < HOLD_STEPS:
            #      _act = torch.zeros_like(_pa)
            #      for _ai, _v in hx_act: _act[0, _ai] = _v
            #  else: _act = _pa
            _act = _pa   # 대기(정지) 중 완전 정지 위해 다리 바이어스 제거(바이어스가 드리프트 유발)
            obs, _, _, _, _ = env.step(_act)

        # ── 시작 대기 중 안정 자세 — 드리프트가 임계 초과할 때만 살짝 재중심(매스텝 텔레포트 금지:
        #  매스텝 write_root_pose 는 물리 솔버와 충돌해 불안정/크래시. 임계 초과시 1회 보정만).
        if STABLE_WAIT and state != "RECOVERY" and not nav_active:
            if math.hypot(rx - SX, ry - SY) > 0.4:     # 시작점에서 0.4m 이상 밀렸을 때만
                with torch.inference_mode():           # 필수: sim write 는 inference_mode 안에서
                    _rsw = robot.data.root_state_w.clone()
                    _rsw[:, 0], _rsw[:, 1] = SX, SY
                    _yw = float(euler_xyz_from_quat(robot.data.root_quat_w[0].unsqueeze(0))[2][0])
                    _rsw[:, 3] = math.cos(_yw / 2.0)
                    _rsw[:, 4] = 0.0
                    _rsw[:, 5] = 0.0
                    _rsw[:, 6] = math.sin(_yw / 2.0)
                    robot.write_root_pose_to_sim(_rsw[:, :7])
                    robot.write_root_velocity_to_sim(torch.zeros_like(_rsw[:, 7:13]))

        # odom/tf (시작점 상대 평면)
        rs = robot.data.root_state_w[0]
        if x0 is None:
            x0, y0 = float(rs[0]), float(rs[1])
        ox, oy = float(rs[0]) - x0, float(rs[1]) - y0
        yaw = float(euler_xyz_from_quat(robot.data.root_quat_w[0].unsqueeze(0))[2][0])
        q = (math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2))
        og.Controller.set(attrs["od_pos"], [ox, oy, 0.0])
        og.Controller.set(attrs["od_ori"], [q[1], q[2], q[3], q[0]])
        og.Controller.set(attrs["tf_t"], [ox, oy, 0.0])
        og.Controller.set(attrs["tf_r"], [q[1], q[2], q[3], q[0]])
        # base_link → spot_cam (손 카메라 FK + optical 보정) — 비전 꼬리 역투영용
        if cam_in_ee_pos is not None:
            _eep = robot.data.body_pos_w[0, ee_idx].unsqueeze(0)
            _eeq = robot.data.body_quat_w[0, ee_idx].unsqueeze(0)
            _cwp = _eep + quat_apply(_eeq, cam_in_ee_pos)
            _cwq = quat_mul(quat_mul(_eeq, cam_in_ee_quat), _Q_RX_PI)   # optical
            _bp = robot.data.root_pos_w[0].unsqueeze(0)
            _bq = robot.data.root_quat_w[0].unsqueeze(0)
            _cb_p = quat_apply_inverse(_bq, _cwp - _bp)[0]
            _cb_q = quat_mul(quat_inv(_bq), _cwq)[0]                    # wxyz
            og.Controller.set(attrs["tfc_t"], [float(_cb_p[0]), float(_cb_p[1]), float(_cb_p[2])])
            og.Controller.set(attrs["tfc_r"],
                              [float(_cb_q[1]), float(_cb_q[2]), float(_cb_q[3]), float(_cb_q[0])])

            # ── in-Isaac 비전 꼬리 검출 → 후방목표 갱신(밖으론 파일만) ──
            if yolo_model is not None and step % VIS_EVERY == 0:
                _tw = detect_tail_world(yolo_model, rgb_annot, depth_annot,
                                        cam_K, _cwp, _cwq, device)
                if _tw is not None:
                    vis_tails.append(_tw)
                    if len(vis_tails) > N_STABLE:
                        vis_tails.pop(0)
                    if len(vis_tails) >= N_STABLE:
                        _tx = float(np.median([p[0] for p in vis_tails]))
                        _ty = float(np.median([p[1] for p in vis_tails]))
                        _ddx, _ddy = _tx - rx, _ty - ry
                        _dd = math.hypot(_ddx, _ddy)
                        if _dd > 1e-3:
                            _ux, _uy = _ddx / _dd, _ddy / _dd
                            _gx = _tx - _ux * BEHIND      # 꼬리에서 로봇쪽 BEHIND(1.5m)
                            _gy = _ty - _uy * BEHIND
                            if (vis_goal_xy is None or math.hypot(
                                    _gx - vis_goal_xy[0], _gy - vis_goal_xy[1]) > RESEND_DIST):
                                vis_goal_xy = (_gx, _gy)
                                try:
                                    import json
                                    with open("/tmp/scenario_goal.json", "w") as _gf:
                                        json.dump({"cow_rear": [_gx - SX, _gy - SY],
                                                   "cow": [_tx - SX, _ty - SY],
                                                   "tail": [_tx - SX, _ty - SY]}, _gf)
                                    print(f"  ★ [비전] 꼬리=({_tx:.2f},{_ty:.2f}) → "
                                          f"후방목표(상대)=({_gx-SX:.2f},{_gy-SY:.2f}) 저장")
                                except Exception as _e:
                                    print(f"  ⚠️ 비전 목표저장: {_e}")

        step += 1
        if step % 120 == 0:
            print(f"  [{state}] pos=({rx:.2f},{ry:.2f}) cmd=({vx:.2f},{wz:.2f})")

    env.close()
    simulation_app.close()


if __name__ == "__main__":
    main()
