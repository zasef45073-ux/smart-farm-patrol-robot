#!/usr/bin/env python3
# Copyright (c) 2024, Smart Farm Patrol Project - 꼬마 로봇 두리
# SPDX-License-Identifier: Apache-2.0
"""
nav_bridge.py
=============
Nav2 폐루프용 Isaac ↔ ROS2 브릿지 (OmniGraph 기반, rclpy 미사용).

이 환경은 Isaac venv=Python3.11, ROS2 Humble=3.10 라 rclpy import 불가.
→ OmniGraph ROS2 노드로 통신하고, 파이썬 메인루프가 노드 속성을 get/set:
   - ROS2SubscribeTwist(/cmd_vel) 출력을 읽어 로봇 kinematic 구동
   - 깨끗한 평면 odom(x,y,yaw) 을 ROS2PublishOdometry(/odom) + RawTF(odom→base_link)
     입력에 써넣어 발행 (시각적 뒤집힘과 ROS 프레임 분리)
   - map→odom static TF (평지: 맵=odom)

실행:
  source ~/dev_ws/venv/isaaclab/bin/activate
  cd ~/dev_ws/isaac_sim/IsaacLab
  ./isaaclab.sh -p ~/dev_ws/spot_ws/src/smart_farm_spot/isaac/nav_bridge.py
"""

import argparse
import math
import os

# ROS2 도메인 강제 (.bashrc 가 0 강제하므로 여기서 시도).
os.environ["ROS_DOMAIN_ID"] = os.environ.get("SF_DOMAIN", "153")

# 워크스페이스 내장 에셋 경로 (포터블: 패키지 상대경로)
_ASSETS = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "assets")

parser = argparse.ArgumentParser()
parser.add_argument("--robot-usd", type=str,
                    default=os.path.join(_ASSETS, "robot", "spot_with_arm.usd"))
parser.add_argument("--robot-prim", type=str, default="/World/Spot")
parser.add_argument("--start-x", type=float, default=0.0)
parser.add_argument("--start-y", type=float, default=0.0)
parser.add_argument("--headless", action="store_true")
a = parser.parse_args()

from isaaclab.app import AppLauncher  # noqa: E402
app = AppLauncher(headless=a.headless).app

from isaacsim.core.utils.extensions import enable_extension  # noqa: E402
enable_extension("omni.graph.action")
enable_extension("isaacsim.ros2.bridge")
enable_extension("omni.replicator.core")
app.update()

import numpy as np  # noqa: E402
import omni.graph.core as og  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.utils.stage import add_reference_to_stage  # noqa: E402
from isaacsim.core.prims import SingleArticulation  # noqa: E402
import isaacsim.core.utils.prims as prim_utils  # noqa: E402

NS_ROS2 = "isaacsim.ros2.bridge"
NS_CORE = "isaacsim.core.nodes"
GRAPH = "/NavBridge"

# 직립/구동 보정 — 초기화(중립): 새 에셋이 똑바로 선 기립 자세 USD 라 보정 불필요.
#  (뒤집힌 구형 에셋용 X180 / yaw180 보정 제거. 필요시 SF_BASE_FLIP=1 로 재활성)
if os.environ.get("SF_BASE_FLIP", "0") == "1":
    BASE_ROT_WXYZ   = (0.0, 1.0, 0.0, 0.0)   # 구형: X180 뒤집힘 보정
    FACE_YAW_OFFSET = 180.0
else:
    BASE_ROT_WXYZ   = (1.0, 0.0, 0.0, 0.0)   # 초기화: 회전 보정 없음
    FACE_YAW_OFFSET = 0.0
SPAWN_Z         = 0.65
SIM_DT          = 1.0 / 60.0

# 에셋 소스 모드 — SF_ASSET_BASE:
#   "local"(기본) = 워크스페이스 assets/scene (서버 불필요, 자립)
#   "omniverse://192.168.10.44/Projects/team1/environment" = Nucleus 서버
_ASSET_BASE = os.environ.get("SF_ASSET_BASE", "local")
if _ASSET_BASE == "local":
    _SCENE = os.path.join(_ASSETS, "scene")
    COW_TEXTURE_BASE = os.path.join(_SCENE, "textures", "cow_fbx",
                                    "RSG_Shepherd_Pack_FBX", "Cow", "Textures")
else:
    _SCENE = _ASSET_BASE.rstrip("/")
    COW_TEXTURE_BASE = f"{_SCENE}/textures"

def _scene(rel):
    return os.path.join(_SCENE, *rel) if _ASSET_BASE == "local" else f"{_SCENE}/{'/'.join(rel)}"

def _asset_exists(p):
    return p.startswith("omniverse://") or os.path.exists(p)

# 축사 환경 (가벼운 모델) — scene/ 안에 props·textures 함께 있어 상대참조 정상
ENV_USD  = _scene(["environment_light.usd"])
ENV_PRIM = "/World/Environment"

# 소 1마리 (ground-truth) — 위치를 map→cow TF 로 발행
#  YOLO(best.pt)가 학습한 cow_eat 모델 사용 (cow_idle 은 인식 안 될 수 있음)
COW_USD  = _scene(["models", "cow_eat.usd"])
COW_PRIM = "/World/Cow"
COW_POS  = (3.0, 0.0, 0.0)   # 로봇 앞 3m

# 센서 — Isaac 기본 RealSense(rsd455) + RTX 라이다
CAM_RES        = (640, 480)
# RealSense 마운트 — 손 엔드단(arm0_link_fngr) 기준 소량 오프셋(손에 밀착).
#  (0.7 은 몸체용이라 손에선 공중으로 튐). SF_RS_POS="x,y,z" 로 조정 가능.
RS_OFFSET_POS  = tuple(float(v) for v in
                       os.environ.get("SF_RS_POS", "0.06,0.0,0.0").split(","))
#  +X(전방) 보게: 수제 카메라에서 검증된 회전값
RS_OFFSET_WXYZ = (0.5, 0.5, -0.5, -0.5)
RS_SUB = "RSD455"
RS_COLOR_REL = f"{RS_SUB}/Camera_OmniVision_OV9782_Color"
RS_DEPTH_REL = f"{RS_SUB}/Camera_Pseudo_Depth"
LIDAR_MOUNT_Z = 0.35
LIDAR_CONFIG  = "Example_Rotary_2D"
TOPIC_RGB   = "/front_stereo_camera/left/image_raw"
TOPIC_DEPTH = "/left/depth"
TOPIC_INFO  = "/front_stereo_camera/left/camera_info"
TOPIC_SCAN  = "/scan"
FRAME_CAM   = "front_cam"
FRAME_LIDAR = "lidar_frame"


def _quat_mul(p, q):
    aw, ax, ay, az = p
    bw, bx, by, bz = q
    return (aw*bw - ax*bx - ay*by - az*bz,
            aw*bx + ax*bw + ay*bz - az*by,
            aw*by - ax*bz + ay*bw + az*bx,
            aw*bz + ax*by - ay*bx + az*bw)


def _yaw_quat_wxyz(yaw):
    h = yaw / 2.0
    return (math.cos(h), 0.0, 0.0, math.sin(h))


def _visual_quat_wxyz(yaw):
    return _quat_mul(_yaw_quat_wxyz(yaw + math.radians(FACE_YAW_OFFSET)), BASE_ROT_WXYZ)


# 소 텍스처(T_Cow_B) 머티리얼 바인딩 — 코드 소환 시 무텍스처(검정) 방지.
#  (scenario1 스포너의 바인딩 로직 이식: UsdPreviewSurface + PNG → 재귀 바인딩)
COW_TEXTURE = (os.path.join(COW_TEXTURE_BASE, "T_Cow_B.png")
              if _ASSET_BASE == "local" else f"{COW_TEXTURE_BASE}/T_Cow_B.png")
COW_MAT_PATH = "/World/Looks/cow_auto"


def _bind_cow_texture(stage, cow_root, texture_path):
    from pxr import Usd, UsdGeom, UsdShade, Sdf
    # 로컬 경로만 존재 확인 (omniverse:// 는 Nucleus 가 해석)
    if not texture_path.startswith("omniverse://") and not os.path.exists(texture_path):
        print(f"  ⚠️ 소 텍스처 없음: {texture_path}")
        return
    if not stage.GetPrimAtPath("/World/Looks"):
        UsdGeom.Scope.Define(stage, "/World/Looks")
    mat = UsdShade.Material.Define(stage, COW_MAT_PATH)
    sh = UsdShade.Shader.Define(stage, f"{COW_MAT_PATH}/PreviewSurface")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.55)
    sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
    tex = UsdShade.Shader.Define(stage, f"{COW_MAT_PATH}/CowTexture")
    tex.CreateIdAttr("UsdUVTexture")
    tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(texture_path))
    tex.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set("sRGB")
    tex.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
    st = UsdShade.Shader.Define(stage, f"{COW_MAT_PATH}/PrimvarReader_st")
    st.CreateIdAttr("UsdPrimvarReader_float2")
    st.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
    st.CreateOutput("result", Sdf.ValueTypeNames.Float2)
    tex.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(st.ConnectableAPI(), "result")
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(tex.ConnectableAPI(), "rgb")
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    root = stage.GetPrimAtPath(cow_root)
    n = 0
    for prim in Usd.PrimRange(root):
        if prim.IsA(UsdGeom.Mesh) or prim.IsA(UsdGeom.Gprim):
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(
                mat, bindingStrength=UsdShade.Tokens.strongerThanDescendants)
            n += 1
    print(f"  ✅ 소 텍스처 바인딩: {n}개 메시 ← {os.path.basename(texture_path)}")


def main():
    print("=" * 60)
    print(" Nav2 OmniGraph 브릿지 (rclpy 미사용)")
    print(f"   ROS_DOMAIN_ID(런타임) = {os.getenv('ROS_DOMAIN_ID')}")
    print("=" * 60)

    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()

    # ── 축사 환경 로드 (가벼운 모델) ─────────────────────────────
    if _asset_exists(ENV_USD):
        add_reference_to_stage(usd_path=ENV_USD, prim_path=ENV_PRIM)
        print(f"  ✅ 축사 환경 로드: {ENV_PRIM} ← {ENV_USD}")
        app.update()
    else:
        print(f"  ⚠️ 환경 USD 없음(바닥만 사용): {ENV_USD}")

    if not _asset_exists(a.robot_usd):
        print(f"❌ 로봇 USD 없음: {a.robot_usd}")
        app.close()
        return
    add_reference_to_stage(usd_path=a.robot_usd, prim_path=a.robot_prim)
    prim_utils.set_prim_attribute_value(
        a.robot_prim, "xformOp:translate", [a.start_x, a.start_y, SPAWN_Z])
    app.update()

    # ── 소 1마리 배치 ─────────────────────────────────────────────
    if _asset_exists(COW_USD):
        import omni.usd
        from pxr import UsdGeom, Gf
        add_reference_to_stage(usd_path=COW_USD, prim_path=COW_PRIM)
        cow_prim = omni.usd.get_context().get_stage().GetPrimAtPath(COW_PRIM)
        xf = UsdGeom.Xformable(cow_prim)
        xf.ClearXformOpOrder()
        xf.AddTranslateOp().Set(Gf.Vec3d(*[float(v) for v in COW_POS]))
        _cow_yaw = float(os.environ.get("SF_COW_YAW", "0"))
        if _cow_yaw:
            xf.AddRotateZOp().Set(_cow_yaw)   # 소 회전(후방/측면 촬영용)
        print(f"  ✅ 소 배치: {COW_PRIM} @ {COW_POS} yaw={_cow_yaw}")
        _bind_cow_texture(omni.usd.get_context().get_stage(), COW_PRIM, COW_TEXTURE)
        app.update()
    else:
        print(f"  ⚠️ 소 USD 없음: {COW_USD}")

    # ── RealSense(rsd455) 부착 + 광축 자동정렬 + RTX 라이다 ────────
    import omni.usd as _omni_usd
    import omni.kit.commands
    from pxr import UsdGeom as _UG, Gf as _Gf, Usd as _Usd
    from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR
    _stage = _omni_usd.get_context().get_stage()

    # 손 엔드단(end-effector) 링크 탐색 — 거기에 카메라 마운트.
    _ee = None
    for _pref in ("arm0_link_fngr", "arm0_link_wr1", "arm0_link_wr0",
                  "arm0_link_el1"):
        for _p in _stage.Traverse():
            if _p.GetName() == _pref:
                _ee = str(_p.GetPath())
                break
        if _ee:
            break
    _ee = _ee or a.robot_prim
    RS_MOUNT = f"{_ee}/RealSense"
    print(f"  ✅ 카메라 마운트(손 엔드단): {RS_MOUNT}")
    add_reference_to_stage(
        usd_path=f"{ISAAC_NUCLEUS_DIR}/Sensors/Intel/RealSense/rsd455.usd",
        prim_path=RS_MOUNT)
    # 손등(그리퍼 위)에 얹기 — 월드 기준 오프셋(위 +Z, 약간 앞)을 손가락 링크
    #  로컬로 변환(로컬 축 추측 제거). SF_RS_WORLD="fwd,side,up" 로 조정.
    _W = np.array([float(v) for v in
                   os.environ.get("SF_RS_WORLD", "0.0,0.0,0.12").split(",")])
    _Mf = _UG.Xformable(_stage.GetPrimAtPath(_ee)).ComputeLocalToWorldTransform(
        _Usd.TimeCode.Default())
    _RM = _Mf.ExtractRotationMatrix()
    _Rf = np.array([[_RM[i][j] for j in range(3)] for i in range(3)], float)  # rows=로컬축(월드)
    _loc = _W @ _Rf.T                                    # 월드→로컬(행벡터)
    _mx = _UG.Xformable(_stage.GetPrimAtPath(RS_MOUNT))
    _mx.ClearXformOpOrder()
    _mx.AddTranslateOp().Set(_Gf.Vec3d(float(_loc[0]), float(_loc[1]), float(_loc[2])))
    print(f"  ✅ 카메라 손등 오프셋: world={_W} → local={_loc.round(3)}")
    _o_op = _mx.AddOrientOp()
    _o_op.Set(_Gf.Quatf(1.0, _Gf.Vec3f(0.0, 0.0, 0.0)))   # 일단 단위
    app.update()
    RS_COLOR = f"{RS_MOUNT}/{RS_COLOR_REL}"
    RS_DEPTH = f"{RS_MOUNT}/{RS_DEPTH_REL}"

    # 자동정렬: 컬러 카메라의 현재 광축(-Z)·up(+Y)을 읽어 +X 전방/+Z up 으로
    #  맞추는 마운트 회전을 계산 (방향 추측 제거).
    _M = _UG.Camera(_stage.GetPrimAtPath(RS_COLOR)).ComputeLocalToWorldTransform(
        _Usd.TimeCode.Default())
    _R = _M.ExtractRotationMatrix()
    _f = np.array([-_R[2][0], -_R[2][1], -_R[2][2]], float)   # forward = -Z (행벡터)
    _u = np.array([_R[1][0], _R[1][1], _R[1][2]], float)      # up = +Y
    _f /= (np.linalg.norm(_f) + 1e-9)
    _u /= (np.linalg.norm(_u) + 1e-9)
    _Mc = np.column_stack([_f, _u, np.cross(_f, _u)])
    _Mt = np.column_stack([[1., 0, 0], [0, 0, 1.], [0, -1., 0]])  # +X, +Z, -Y
    _Rm = _Mt @ _Mc.T
    _tr = _Rm[0, 0] + _Rm[1, 1] + _Rm[2, 2]
    if _tr > 0:
        _s = math.sqrt(_tr + 1.0) * 2
        _qw = 0.25 * _s
        _qx = (_Rm[2, 1] - _Rm[1, 2]) / _s
        _qy = (_Rm[0, 2] - _Rm[2, 0]) / _s
        _qz = (_Rm[1, 0] - _Rm[0, 1]) / _s
    elif _Rm[0, 0] > _Rm[1, 1] and _Rm[0, 0] > _Rm[2, 2]:
        _s = math.sqrt(1.0 + _Rm[0, 0] - _Rm[1, 1] - _Rm[2, 2]) * 2
        _qw = (_Rm[2, 1] - _Rm[1, 2]) / _s
        _qx = 0.25 * _s
        _qy = (_Rm[0, 1] + _Rm[1, 0]) / _s
        _qz = (_Rm[0, 2] + _Rm[2, 0]) / _s
    elif _Rm[1, 1] > _Rm[2, 2]:
        _s = math.sqrt(1.0 + _Rm[1, 1] - _Rm[0, 0] - _Rm[2, 2]) * 2
        _qw = (_Rm[0, 2] - _Rm[2, 0]) / _s
        _qx = (_Rm[0, 1] + _Rm[1, 0]) / _s
        _qy = 0.25 * _s
        _qz = (_Rm[1, 2] + _Rm[2, 1]) / _s
    else:
        _s = math.sqrt(1.0 + _Rm[2, 2] - _Rm[0, 0] - _Rm[1, 1]) * 2
        _qw = (_Rm[1, 0] - _Rm[0, 1]) / _s
        _qx = (_Rm[0, 2] + _Rm[2, 0]) / _s
        _qy = (_Rm[1, 2] + _Rm[2, 1]) / _s
        _qz = 0.25 * _s
    # SF_RS_QUAT="w,x,y,z" 로 수동 오버라이드 가능
    _env_q = os.environ.get("SF_RS_QUAT")
    if _env_q:
        _qw, _qx, _qy, _qz = [float(v) for v in _env_q.split(",")]
    _o_op.Set(_Gf.Quatf(float(_qw), _Gf.Vec3f(float(_qx), float(_qy), float(_qz))))
    app.update()
    print(f"  ✅ RealSense 자동정렬: fwd0={_f.round(2)} quat=({_qw:.3f},{_qx:.3f},{_qy:.3f},{_qz:.3f})")

    # RealSense 카메라 클리핑 범위 교정 — 기본 far=0.3m 라 그 너머가 전부 검정.
    #  → 0.01~1000m 로 늘려 씬(소 3m, 축사)이 보이게.
    for _cp in (RS_COLOR, RS_DEPTH, f"{RS_MOUNT}/{RS_SUB}/Camera_OmniVision_OV9782_Left",
                f"{RS_MOUNT}/{RS_SUB}/Camera_OmniVision_OV9782_Right"):
        _cprim = _stage.GetPrimAtPath(_cp)
        if _cprim and _cprim.IsValid():
            _UG.Camera(_cprim).GetClippingRangeAttr().Set(_Gf.Vec2f(0.01, 1000.0))
    print("  ✅ RealSense 클리핑 0.01~1000m 교정")

    LIDAR_MOUNT = f"{a.robot_prim}/Lidar360"
    _lm = _UG.Xform.Define(_stage, LIDAR_MOUNT)
    _lx = _UG.Xformable(_lm.GetPrim())
    _lx.ClearXformOpOrder()
    _lx.AddTranslateOp().Set(_Gf.Vec3d(0.0, 0.0, LIDAR_MOUNT_Z))
    LIDAR_PRIM = None
    try:
        _, _lidar = omni.kit.commands.execute(
            "IsaacSensorCreateRtxLidar", path="/lidar_sensor",
            parent=LIDAR_MOUNT, config=LIDAR_CONFIG)
        LIDAR_PRIM = f"{LIDAR_MOUNT}/lidar_sensor"
        print(f"  ✅ RTX Lidar: {LIDAR_PRIM} (config={LIDAR_CONFIG})")
    except Exception as e:
        print(f"  ❌ Lidar 생성 실패: {e}")
    app.update()

    robot = SingleArticulation(prim_path=a.robot_prim, name="spot")
    world.reset()
    robot.initialize()

    # ── 렌더프로덕트 (카메라 1개 → RGB/뎁스 공용 + 라이다) ─────────
    import omni.replicator.core as rep
    _cam_rp = rep.create.render_product(RS_COLOR, CAM_RES)
    RGB_RP_PATH = _cam_rp.path if hasattr(_cam_rp, "path") else str(_cam_rp)
    DEPTH_RP_PATH = RGB_RP_PATH   # 같은 렌더프로덕트로 뎁스도 발행
    print("  ✅ RGB/뎁스 렌더프로덕트 생성")
    LIDAR_RP_PATH = None
    if LIDAR_PRIM is not None:
        try:
            _lidar_rp = rep.create.render_product(LIDAR_PRIM, [1, 1])
            LIDAR_RP_PATH = _lidar_rp.path if hasattr(_lidar_rp, "path") else str(_lidar_rp)
        except Exception as e:
            print(f"  ❌ Lidar 렌더프로덕트 실패: {e}")

    # ── 시작 자세 정의 (펴쳐진 채 시작 → 팔만 접기 → 주행) ──────────
    #  이 USD 는 다리=0 이 "펴쳐진(extended) 기립". 기본 관절값은 팔 펼침.
    _jn = list(robot.dof_names)
    print(f"  ✅ dof_names({len(_jn)}): {_jn}")
    _default = np.array(robot.get_joint_positions(), dtype=float).copy()
    leg_ids = [i for i, n in enumerate(_jn)
               if any(s in n for s in ("_hx", "_hy", "_kn"))]
    # 모든 관절 각도 0.0 으로 초기화 (새 에셋: 관절 0 = 기립 자세)
    ARM_FOLD = {}                       # 팔 접힘 각도 없음(0 유지)
    START_POSE = np.zeros(len(_jn), dtype=float)
    DRIVE_POSE = np.zeros(len(_jn), dtype=float)
    print(f"  ✅ 자세 준비: 전 관절 0.0 초기화 ({len(_jn)}관절)")

    # 시퀀스 타이밍(스텝, 60fps): 펴진 채 유지 → 팔접기
    HOLD, FOLD_DUR = 60, 120
    SEQ_END = HOLD + FOLD_DUR

    def _lerp(a, b, t):
        return a + (b - a) * max(0.0, min(1.0, t))

    # ── 기립 베이스 높이 (고정, 시각 튜닝값) ────────────────────────
    #  자동 보정이 이 래퍼 USD 충돌과 충돌해 비정상값(-0.1 등) → 고정 사용.
    #  떠있으면 낮추고, 박히면 올린다. SF_STAND_Z 로 재정의 가능.
    STAND_Z = float(os.environ.get("SF_STAND_Z", "0.60"))
    print(f"  ✅ 기립 높이(고정): STAND_Z={STAND_Z:.3f}")

    # 내부 평면 상태 (펴쳐진 채 시작)
    x, y, yaw = float(a.start_x), float(a.start_y), 0.0
    robot.set_world_pose(position=np.array([x, y, STAND_Z]),
                         orientation=np.array(_visual_quat_wxyz(yaw)))
    robot.set_joint_positions(START_POSE)
    app.update()

    # ── OmniGraph: clock + cmd_vel 구독 + odom/tf 발행 ────────────
    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": GRAPH, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("Tick",     "omni.graph.action.OnPlaybackTick"),
                ("SimTime",  f"{NS_CORE}.IsaacReadSimulationTime"),
                ("ClockPub", f"{NS_ROS2}.ROS2PublishClock"),
                ("SubTwist", f"{NS_ROS2}.ROS2SubscribeTwist"),
                ("OdomPub",  f"{NS_ROS2}.ROS2PublishOdometry"),
                ("TfOdom",   f"{NS_ROS2}.ROS2PublishRawTransformTree"),
                ("TfMap",    f"{NS_ROS2}.ROS2PublishRawTransformTree"),
                ("TfCow",    f"{NS_ROS2}.ROS2PublishRawTransformTree"),
                ("RgbPub",   f"{NS_ROS2}.ROS2CameraHelper"),
                ("DepthPub", f"{NS_ROS2}.ROS2CameraHelper"),
                ("InfoPub",  f"{NS_ROS2}.ROS2CameraInfoHelper"),
                ("LidarPub", f"{NS_ROS2}.ROS2RtxLidarHelper"),
            ],
            keys.SET_VALUES: [
                ("ClockPub.inputs:topicName", "/clock"),
                ("SubTwist.inputs:topicName", "/cmd_vel"),
                # RealSense: RGB / 뎁스 / camera_info
                ("RgbPub.inputs:renderProductPath",   RGB_RP_PATH),
                ("RgbPub.inputs:type",       "rgb"),
                ("RgbPub.inputs:topicName",  TOPIC_RGB),
                ("RgbPub.inputs:frameId",    FRAME_CAM),
                ("DepthPub.inputs:renderProductPath", DEPTH_RP_PATH),
                ("DepthPub.inputs:type",     "depth"),
                ("DepthPub.inputs:topicName", TOPIC_DEPTH),
                ("DepthPub.inputs:frameId",  FRAME_CAM),
                ("InfoPub.inputs:renderProductPath",  RGB_RP_PATH),
                ("InfoPub.inputs:topicName", TOPIC_INFO),
                ("InfoPub.inputs:frameId",   FRAME_CAM),
                # RTX 라이다 → /scan (LaserScan)
                ("LidarPub.inputs:renderProductPath", LIDAR_RP_PATH or ""),
                ("LidarPub.inputs:topicName", TOPIC_SCAN),
                ("LidarPub.inputs:frameId",   FRAME_LIDAR),
                ("LidarPub.inputs:type",      "laser_scan"),
                ("OdomPub.inputs:topicName",     "/odom"),
                ("OdomPub.inputs:odomFrameId",   "odom"),
                ("OdomPub.inputs:chassisFrameId", "base_link"),
                ("TfOdom.inputs:topicName",      "/tf"),
                ("TfOdom.inputs:parentFrameId",  "odom"),
                ("TfOdom.inputs:childFrameId",   "base_link"),
                ("TfMap.inputs:topicName",       "/tf_static"),
                ("TfMap.inputs:parentFrameId",   "map"),
                ("TfMap.inputs:childFrameId",    "odom"),
                ("TfMap.inputs:staticPublisher", True),
                ("TfCow.inputs:topicName",       "/tf_static"),
                ("TfCow.inputs:parentFrameId",   "map"),
                ("TfCow.inputs:childFrameId",    "cow"),
                ("TfCow.inputs:translation",     list(COW_POS)),
                ("TfCow.inputs:staticPublisher", True),
            ],
            keys.CONNECT: [
                ("Tick.outputs:tick", "ClockPub.inputs:execIn"),
                ("Tick.outputs:tick", "SubTwist.inputs:execIn"),
                ("Tick.outputs:tick", "OdomPub.inputs:execIn"),
                ("Tick.outputs:tick", "TfOdom.inputs:execIn"),
                ("Tick.outputs:tick", "TfMap.inputs:execIn"),
                ("Tick.outputs:tick", "TfCow.inputs:execIn"),
                ("Tick.outputs:tick", "RgbPub.inputs:execIn"),
                ("Tick.outputs:tick", "DepthPub.inputs:execIn"),
                ("Tick.outputs:tick", "InfoPub.inputs:execIn"),
                ("Tick.outputs:tick", "LidarPub.inputs:execIn"),
                ("SimTime.outputs:simulationTime", "ClockPub.inputs:timeStamp"),
                ("SimTime.outputs:simulationTime", "OdomPub.inputs:timeStamp"),
                ("SimTime.outputs:simulationTime", "TfOdom.inputs:timeStamp"),
                ("SimTime.outputs:simulationTime", "TfMap.inputs:timeStamp"),
                ("SimTime.outputs:simulationTime", "TfCow.inputs:timeStamp"),
            ],
        },
    )
    print("  ✅ OmniGraph: /clock, /cmd_vel(sub), /odom, /tf(odom→base, map→odom)")

    A = og.Controller.attribute
    sub_lin = A(f"{GRAPH}/SubTwist.outputs:linearVelocity")
    sub_ang = A(f"{GRAPH}/SubTwist.outputs:angularVelocity")
    od_pos  = A(f"{GRAPH}/OdomPub.inputs:position")
    od_ori  = A(f"{GRAPH}/OdomPub.inputs:orientation")
    od_lin  = A(f"{GRAPH}/OdomPub.inputs:linearVelocity")
    od_ang  = A(f"{GRAPH}/OdomPub.inputs:angularVelocity")
    tf_t    = A(f"{GRAPH}/TfOdom.inputs:translation")
    tf_r    = A(f"{GRAPH}/TfOdom.inputs:rotation")

    print("\n[실행] Nav2 브릿지 동작. 다른 터미널(시스템 ROS2, 같은 도메인):")
    print("   ros2 topic echo /odom   /   ros2 topic pub /cmd_vel ...\n")

    step = 0
    try:
        while app.is_running():
            # ── 시작 시퀀스: 펴쳐진 유지 → 팔접기 → (주행) ──
            if step < SEQ_END:
                if step < HOLD:                      # ① 펴쳐진 채 유지
                    jpos, phase = START_POSE, "펴쳐짐"
                else:                                # ② 팔 접기
                    t = (step - HOLD) / FOLD_DUR
                    jpos, phase = _lerp(START_POSE, DRIVE_POSE, t), "팔접기"
                cmd_vx = cmd_wz = 0.0                # 시퀀스 중 주행 정지
                robot.set_world_pose(position=np.array([x, y, STAND_Z]),
                                     orientation=np.array(_visual_quat_wxyz(yaw)))
                robot.set_joint_positions(jpos)
                robot.set_linear_velocity(np.zeros(3))
                robot.set_angular_velocity(np.zeros(3))
                if step % 30 == 0:
                    print(f"  [시퀀스] {phase} ({step}/{SEQ_END})")
            else:
                # ── /cmd_vel 읽기 (직전 틱 수신값) ──
                lin = og.Controller.get(sub_lin)
                ang = og.Controller.get(sub_ang)
                cmd_vx = float(lin[0]) if lin is not None else 0.0
                cmd_wz = float(ang[2]) if ang is not None else 0.0

                # ── kinematic 구동: 내부 적분(드리프트 없음) + 텔레포트 ──
                yaw = math.atan2(math.sin(yaw + cmd_wz * SIM_DT),
                                 math.cos(yaw + cmd_wz * SIM_DT))
                x += cmd_vx * math.cos(yaw) * SIM_DT
                y += cmd_vx * math.sin(yaw) * SIM_DT
                robot.set_world_pose(position=np.array([x, y, STAND_Z]),
                                     orientation=np.array(_visual_quat_wxyz(yaw)))
                robot.set_linear_velocity(np.zeros(3))   # 잔여속도 제거
                robot.set_angular_velocity(np.zeros(3))
                robot.set_joint_positions(DRIVE_POSE)    # 펴쳐진 기립+팔접힘 유지

            # ── 깨끗한 odom/TF 입력 써넣기 (IJKR = x,y,z,w) ──
            qw, qx, qy, qz = _yaw_quat_wxyz(yaw)
            og.Controller.set(od_pos, [x, y, 0.0])
            og.Controller.set(od_ori, [qx, qy, qz, qw])
            og.Controller.set(od_lin, [cmd_vx, 0.0, 0.0])
            og.Controller.set(od_ang, [0.0, 0.0, cmd_wz])
            og.Controller.set(tf_t, [x, y, 0.0])
            og.Controller.set(tf_r, [qx, qy, qz, qw])

            world.step(render=True)
            step += 1
            if step % 240 == 0:
                print(f"  ... {step}스텝 | cmd=({cmd_vx:.2f},{cmd_wz:.2f}) "
                      f"pose=({x:.2f},{y:.2f},{math.degrees(yaw):.0f}°)")
    except KeyboardInterrupt:
        print("\n[종료]")
    finally:
        app.close()


if __name__ == "__main__":
    main()
