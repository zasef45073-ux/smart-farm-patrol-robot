#!/usr/bin/env python3
# Copyright (c) 2024, Smart Farm Patrol Project - 꼬마 로봇 두리
# SPDX-License-Identifier: Apache-2.0
"""
camera_record.py
================
헤드리스로 로봇 손 끝단 RealSense(RGB+Depth, 320x320 고정)를 녹화.
  · mp4 영상(인프로세스 annotator → cv2): ~/spot_cam_rgb.mp4, ~/spot_cam_depth.mp4
  · ROS2 토픽 발행(/spot_cam/rgb, /depth, /camera_info, /clock) → 외부 `ros2 bag record`로 rosbag

특징:
  · 320x320 전부 고정(RGB·Depth 렌더프로덕트 동일 해상도)
  · 뎁스 최적화: 클리핑 (0.05~10m), distance_to_image_plane 미터단위
  · 험지 terrain + 자동리셋 OFF → 로봇 안정(텔레포트 없음)
  · 명령 화살표(velocity 시각화)는 디버그용으로 유지

실행:
  source ~/dev_ws/venv/isaaclab/bin/activate
  cd ~/dev_ws/isaac_sim/IsaacLab
  ./isaaclab.sh -p ~/dev_ws/spot_ws/src/smart_farm_spot/isaac/camera_record.py
"""

import argparse
import datetime
import math
import os

os.environ["ROS_DOMAIN_ID"] = os.environ.get("ROS_DOMAIN_ID", "153")

_ASSETS = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "assets")
TASK = "Isaac-Velocity-Rough-SpotArm-Play-v0"
# 정책: SF_POLICY 로 지정(안정 검증판 권장: spot_flat/2026-06-07_14-01-53/exported/policy.pt).
#  최신이 더 좋은 게 아님 — 안정 버전 고정. 이 파일은 읽기만(절대 변경 안 함).
POLICY = os.environ.get("SF_POLICY", os.path.join(_ASSETS, "policy", "policy.pt"))
ENV_USD = os.path.join(_ASSETS, "scene", "environment_final.usd")
HOME = os.path.expanduser("~")

RES = (320, 320)                                   # 전부 고정 320x320
SECONDS = float(os.environ.get("SF_REC_SECONDS", "20"))
FPS = int(os.environ.get("SF_REC_FPS", "30"))
DEPTH_NEAR = float(os.environ.get("SF_DEPTH_NEAR", "0.05"))   # 뎁스 최적 범위
DEPTH_FAR = float(os.environ.get("SF_DEPTH_FAR", "10.0"))
LOAD_BARN = os.environ.get("SF_BARN", "1") == "1"
PEN_FRICTION = float(os.environ.get("SF_PEN_FRICTION", "0.45"))       # 우리(저)
CORRIDOR_FRICTION = float(os.environ.get("SF_CORRIDOR_FRICTION", "0.7"))  # 통로(고)

# ── 시간·날짜 버전관리 출력 폴더 ─────────────────────────────────────
#  매 녹화를 ~/rag/<YYYYMMDD_HHMMSS>/ 로 분리 저장(덮어쓰기 방지).
#  SF_OUT_DIR 로 외부(rosbag 동일 폴더)와 동기화 가능.
_TS = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
OUT_DIR = os.environ.get("SF_OUT_DIR", os.path.join(HOME, "rag", _TS))

parser = argparse.ArgumentParser()
parser.add_argument("--task", default=TASK)
parser.add_argument("--policy", default=POLICY)
a = parser.parse_args()

from isaaclab.app import AppLauncher  # noqa: E402
# 기본 GUI(직접 디버깅). SF_HEADLESS=1 이면 헤드리스. 카메라 파이프라인 위해 enable_cameras 유지.
_HEADLESS = os.environ.get("SF_HEADLESS", "0") == "1"
app_launcher = AppLauncher(headless=_HEADLESS, enable_cameras=True)
simulation_app = app_launcher.app

# 헤드리스 experience 엔 omni.graph/ros2 가 기본 미로드 → 먼저 활성화 후 import
from isaacsim.core.utils.extensions import enable_extension  # noqa: E402
for _ext in ("omni.graph.core", "omni.graph.action", "omni.graph.nodes",
             "isaacsim.core.nodes", "isaacsim.ros2.bridge",
             "omni.replicator.core"):
    try:
        enable_extension(_ext)
    except Exception as _e:
        print(f"  ⚠️ ext {_ext}: {_e}")
simulation_app.update()

import numpy as np  # noqa: E402
import torch  # noqa: E402
import cv2  # noqa: E402
import gymnasium as gym  # noqa: E402
import omni.graph.core as og  # noqa: E402
import omni.kit.commands  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
import omni.replicator.core as rep  # noqa: E402
from pxr import Gf, Usd, UsdGeom, UsdPhysics, UsdShade  # noqa: E402
from isaacsim.core.utils.stage import add_reference_to_stage  # noqa: E402
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR  # noqa: E402
import isaaclab_tasks  # noqa: F401,E402
from isaaclab_tasks.utils import parse_env_cfg  # noqa: E402

NS_ROS2 = "isaacsim.ros2.bridge"
NS_CORE = "isaacsim.core.nodes"
GRAPH = "/CameraRecordGraph"
RS_SUB = "RSD455"
RS_COLOR_REL = f"{RS_SUB}/Camera_OmniVision_OV9782_Color"
RS_DEPTH_REL = f"{RS_SUB}/Camera_Pseudo_Depth"


def mount_realsense(stage, robot_prim):
    """손 엔드단(arm0_link_fngr)에 RealSense 부착 + 손등 오프셋 + 광축 자동정렬 + 클립."""
    _ee = None
    for _pref in ("arm0_link_fngr", "arm0_link_wr1", "arm0_link_wr0", "arm0_link_el1"):
        for _p in stage.Traverse():
            if _p.GetName() == _pref:
                _ee = str(_p.GetPath()); break
        if _ee:
            break
    _ee = _ee or robot_prim
    RS_MOUNT = f"{_ee}/RealSense"
    print(f"  ✅ 카메라 마운트(손 엔드단): {RS_MOUNT}")
    add_reference_to_stage(
        usd_path=f"{ISAAC_NUCLEUS_DIR}/Sensors/Intel/RealSense/rsd455.usd",
        prim_path=RS_MOUNT)
    # 손등(그리퍼 위) 오프셋: 월드(위 +Z, 약간 앞) → 로컬 변환
    _W = np.array([float(v) for v in
                   os.environ.get("SF_RS_WORLD", "0.0,0.0,0.06").split(",")])
    _Mf = UsdGeom.Xformable(stage.GetPrimAtPath(_ee)).ComputeLocalToWorldTransform(
        Usd.TimeCode.Default())
    _RM = _Mf.ExtractRotationMatrix()
    _Rf = np.array([[_RM[i][j] for j in range(3)] for i in range(3)], float)
    _loc = _W @ _Rf.T
    _mx = UsdGeom.Xformable(stage.GetPrimAtPath(RS_MOUNT))
    _mx.ClearXformOpOrder()
    _mx.AddTranslateOp().Set(Gf.Vec3d(float(_loc[0]), float(_loc[1]), float(_loc[2])))
    _o_op = _mx.AddOrientOp()
    _o_op.Set(Gf.Quatf(1.0, Gf.Vec3f(0.0, 0.0, 0.0)))
    simulation_app.update()

    RS_COLOR = f"{RS_MOUNT}/{RS_COLOR_REL}"
    RS_DEPTH = f"{RS_MOUNT}/{RS_DEPTH_REL}"
    # 자동정렬: 컬러 광축(-Z)/up(+Y) → +X 전방/+Z up
    _M = UsdGeom.Camera(stage.GetPrimAtPath(RS_COLOR)).ComputeLocalToWorldTransform(
        Usd.TimeCode.Default())
    _R = _M.ExtractRotationMatrix()
    _f = np.array([-_R[2][0], -_R[2][1], -_R[2][2]], float)
    _u = np.array([_R[1][0], _R[1][1], _R[1][2]], float)
    _f /= (np.linalg.norm(_f) + 1e-9)
    _u /= (np.linalg.norm(_u) + 1e-9)
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
    _env_q = os.environ.get("SF_RS_QUAT")
    if _env_q:
        _qw, _qx, _qy, _qz = [float(v) for v in _env_q.split(",")]
    _o_op.Set(Gf.Quatf(float(_qw), Gf.Vec3f(float(_qx), float(_qy), float(_qz))))
    simulation_app.update()
    print(f"  ✅ RealSense 자동정렬 quat=({_qw:.3f},{_qx:.3f},{_qy:.3f},{_qz:.3f})")

    # 클립: 컬러는 넓게(0.01~1000), 뎁스는 최적 범위(near~far)
    for _cp in (RS_COLOR, f"{RS_MOUNT}/{RS_SUB}/Camera_OmniVision_OV9782_Left",
                f"{RS_MOUNT}/{RS_SUB}/Camera_OmniVision_OV9782_Right"):
        _c = stage.GetPrimAtPath(_cp)
        if _c and _c.IsValid():
            UsdGeom.Camera(_c).GetClippingRangeAttr().Set(Gf.Vec2f(0.01, 1000.0))
    _dp = stage.GetPrimAtPath(RS_DEPTH)
    if _dp and _dp.IsValid():
        UsdGeom.Camera(_dp).GetClippingRangeAttr().Set(Gf.Vec2f(DEPTH_NEAR, DEPTH_FAR))
    print(f"  ✅ 클립: 컬러 0.01~1000m / 뎁스 최적 {DEPTH_NEAR}~{DEPTH_FAR}m")
    return RS_COLOR, RS_DEPTH


def apply_zone_friction(stage, barn_root, pen_mu, corridor_mu):
    """축사 바닥 메시에 **구역별 마찰** physics material + 콜라이더 부여.
      Bedding*(배변 우리)=pen_mu 저마찰 / Sand*Corridor·Ground(통로)=corridor_mu 고마찰.
      events.physics_material=None 이면 로봇 발 접촉마찰 = 이 구역 μ (eval_h_course_nav 방식).
    """
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
    n_pen = n_cor = 0
    for p in Usd.PrimRange(root):
        if p.GetTypeName() != "Mesh":
            continue
        nm = p.GetName().lower()
        if "bedding" in nm:
            m, _t = pen_mat, "pen"
        elif ("sand" in nm and "corridor" in nm) or nm == "ground_mesh" or "floor" in nm:
            m, _t = cor_mat, "cor"
        else:
            continue
        UsdPhysics.CollisionAPI.Apply(p)
        b = UsdShade.MaterialBindingAPI.Apply(p)
        b.Bind(m, bindingStrength=UsdShade.Tokens.strongerThanDescendants,
               materialPurpose="physics")
        if _t == "pen":
            n_pen += 1
        else:
            n_cor += 1
    print(f"  ✅ 구역마찰: 우리(Bedding) μ={pen_mu}×{n_pen} / 통로(Sand) μ={corridor_mu}×{n_cor}")
    return n_pen + n_cor


def apply_wall_collision(stage, barn_root):
    """축사 가상벽(Invisible_Escape_Barriers)·펜스·벽 메시에 콜라이더 부여.
      → 로봇이 통과 못 하게(물리 충돌 방지). 라이다는 시각 메시를 보고 Nav2가 회피.
    """
    root = stage.GetPrimAtPath(barn_root)
    if not root or not root.IsValid():
        return 0
    kws = ("barrier", "escape", "fence", "wall", "rail")
    geom = ("Mesh", "Cube", "Cylinder", "Capsule", "Cone", "Sphere")
    cand = []
    n = 0
    for p in Usd.PrimRange(root):
        path = str(p.GetPath()).lower()
        if any(k in path for k in kws):
            cand.append((p.GetTypeName(), str(p.GetPath())))
            if p.GetTypeName() in geom:
                UsdPhysics.CollisionAPI.Apply(p)
                n += 1
    print(f"  ✅ 가상벽/펜스 충돌 콜라이더: {n}개 적용 (매칭 prim {len(cand)})")
    for _t, _p in cand[:10]:
        print(f"      [{_t}] {_p}")
    # 진단: 매칭 0 이면 축사 그룹(Xform/Scope) 이름 덤프
    if not cand:
        print("  [진단] 축사 그룹(Xform/Scope) 이름:")
        seen = 0
        for p in Usd.PrimRange(root):
            if p.GetTypeName() in ("Xform", "Scope") and seen < 25:
                print(f"      {p.GetPath()}")
                seen += 1
    return n


def build_ros_graph(rp_color_path, rp_depth_path):
    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": GRAPH, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("Tick", "omni.graph.action.OnPlaybackTick"),
                ("SimTime", f"{NS_CORE}.IsaacReadSimulationTime"),
                ("ClockPub", f"{NS_ROS2}.ROS2PublishClock"),
                ("RgbPub", f"{NS_ROS2}.ROS2CameraHelper"),
                ("DepthPub", f"{NS_ROS2}.ROS2CameraHelper"),
                ("InfoPub", f"{NS_ROS2}.ROS2CameraInfoHelper"),
            ],
            keys.SET_VALUES: [
                ("ClockPub.inputs:topicName", "/clock"),
                ("RgbPub.inputs:renderProductPath", rp_color_path),
                ("RgbPub.inputs:type", "rgb"),
                ("RgbPub.inputs:topicName", "/spot_cam/rgb"),
                ("RgbPub.inputs:frameId", "spot_cam"),
                ("DepthPub.inputs:renderProductPath", rp_depth_path),
                ("DepthPub.inputs:type", "depth"),
                ("DepthPub.inputs:topicName", "/spot_cam/depth"),
                ("DepthPub.inputs:frameId", "spot_cam"),
                ("InfoPub.inputs:renderProductPath", rp_color_path),
                ("InfoPub.inputs:topicName", "/spot_cam/camera_info"),
                ("InfoPub.inputs:frameId", "spot_cam"),
            ],
            keys.CONNECT: [
                ("Tick.outputs:tick", "ClockPub.inputs:execIn"),
                ("Tick.outputs:tick", "RgbPub.inputs:execIn"),
                ("Tick.outputs:tick", "DepthPub.inputs:execIn"),
                ("Tick.outputs:tick", "InfoPub.inputs:execIn"),
                ("SimTime.outputs:simulationTime", "ClockPub.inputs:timeStamp"),
            ],
        },
    )
    print("  ✅ ROS2: /clock /spot_cam/rgb /spot_cam/depth /spot_cam/camera_info")


def main():
    print("=" * 60)
    print(" 헤드리스 손 RealSense 녹화 (320x320, RGB+Depth)")
    print(f"   {SECONDS:.0f}s @ {FPS}fps → mp4 + ROS토픽(rosbag)")
    print("=" * 60)

    env_cfg = parse_env_cfg(a.task, device="cuda:0", num_envs=1)
    # 자동 리셋 끔(텔레포트 방지)
    try:
        for _n in [n for n in vars(env_cfg.terminations) if not n.startswith("_")]:
            setattr(env_cfg.terminations, _n, None)
        print("  ✅ 자동 리셋(termination) 끔")
    except Exception as e:
        print(f"  ⚠️ termination: {e}")
    # 명령 화살표(velocity 시각화)는 디버그용으로 유지 — debug_vis 끄지 않음.
    print("  ✅ 명령 화살표 디버그 표시 유지")

    # 평면 + 구역별 마찰(eval_h_course_nav 방식) — 학습험지 불필요, 평면정책 사용.
    #  height_scan 은 평면(/World/ground)만, 발 접촉마찰은 축사 바닥 구역 μ. SF_ROUGH=1 이면 험지 유지.
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
            env_cfg.events.physics_material = None   # 접촉마찰 = 축사 구역 μ
        except Exception:
            pass
        print("  ✅ 평면 + height_scan=/World/ground + 구역마찰(접촉) 모드")

    env = gym.make(a.task, cfg=env_cfg)
    obs, _ = env.reset()
    device = env.unwrapped.device
    stage = omni.usd.get_context().get_stage()
    robot = env.unwrapped.scene["robot"]
    robot_prim = robot.root_physx_view.prim_paths[0]

    # 정책 로드(워밍업에 필요)
    policy = torch.jit.load(a.policy, map_location=device)
    policy.eval()
    cmd_term = env.unwrapped.command_manager.get_term("base_velocity")
    try:
        cmd_term.cfg.resampling_time_range = (1.0e9, 1.0e9)
        cmd_term.cfg.heading_command = False
        cmd_term.vel_command_b[:] = 0.0
    except Exception:
        pass

    # 워밍업: 정책으로 안착(스폰은 공중→낙하 후 험지에 발 안착) 후 발높이 측정
    for _ in range(60):
        with torch.inference_mode():
            _o = obs["policy"] if isinstance(obs, dict) else obs
            obs, _, _, _, _ = env.step(policy(_o))

    _rs = robot.data.root_state_w[0]
    SX, SY, SZ = float(_rs[0]), float(_rs[1]), float(_rs[2])
    # 발(최저 body) 실제 높이를 축사 바닥으로 → 박힘 방지
    try:
        _fz = float(robot.data.body_pos_w[0, :, 2].min())
    except Exception:
        _fz = SZ - 0.55
    # 발 링크 원점은 실제 접촉점보다 ~0.20 위 → 그만큼 빼야 축사바닥이 발바닥에 맞음(박힘 방지)
    FLOOR_Z = _fz - float(os.environ.get("SF_FOOT_OFFSET", "0.20")) \
        + float(os.environ.get("SF_FLOOR_OFFSET", "0.0"))
    print(f"  ✅ 로봇 안착: ({SX:.2f},{SY:.2f},{SZ:.2f}) 발링크={_fz:.3f} → 축사바닥 z={FLOOR_Z:.3f}")

    # 축사(시각 배경)
    if LOAD_BARN and os.path.exists(ENV_USD):
        try:
            add_reference_to_stage(usd_path=ENV_USD, prim_path="/World/Barn")
            _bx = UsdGeom.Xformable(stage.GetPrimAtPath("/World/Barn"))
            _bx.ClearXformOpOrder()
            _bx.AddTranslateOp().Set(Gf.Vec3d(SX, SY, FLOOR_Z))
            simulation_app.update()
            print("  ✅ 축사 배경 로드")
            # 구역별 마찰(평면 모드에서만 의미 — 발이 축사 바닥에 접촉)
            if os.environ.get("SF_ROUGH", "0") != "1":
                apply_zone_friction(stage, "/World/Barn",
                                    PEN_FRICTION, CORRIDOR_FRICTION)
            # 가상벽/펜스 충돌(통과 방지) — 험지/평면 무관 적용
            apply_wall_collision(stage, "/World/Barn")
        except Exception as e:
            print(f"  ⚠️ 축사: {e}")

    # IsaacLab 평면 격자(grid) 시각 숨김 → 맵이 축사로 보이게. height_scan/충돌은 유지.
    if os.environ.get("SF_HIDE_GRID", "1") == "1":
        for _gp in ("/World/ground", "/World/GroundPlane", "/World/defaultGroundPlane"):
            _g = stage.GetPrimAtPath(_gp)
            if _g and _g.IsValid():
                try:
                    UsdGeom.Imageable(_g).MakeInvisible()
                    print(f"  ✅ 평면 격자 시각 숨김: {_gp}")
                except Exception as e:
                    print(f"  ⚠️ 격자 숨김({_gp}): {e}")

    RS_COLOR, RS_DEPTH = mount_realsense(stage, robot_prim)

    # 320x320 렌더프로덕트 (RGB·Depth 동일 해상도 고정)
    rp_color = rep.create.render_product(RS_COLOR, RES)
    rp_depth = rep.create.render_product(RS_DEPTH, RES)
    rp_color_path = rp_color.path if hasattr(rp_color, "path") else str(rp_color)
    rp_depth_path = rp_depth.path if hasattr(rp_depth, "path") else str(rp_depth)
    print(f"  ✅ 렌더프로덕트 320x320: color={rp_color_path}")

    # annotators (인프로세스 mp4용)
    rgb_an = rep.AnnotatorRegistry.get_annotator("rgb")
    rgb_an.attach(rp_color)
    depth_an = rep.AnnotatorRegistry.get_annotator("distance_to_image_plane")
    depth_an.attach(rp_depth)

    # ROS2 발행(rosbag용) — 정책·cmd_term 은 위에서 이미 준비됨
    enable_extension("isaacsim.ros2.bridge")
    simulation_app.update()
    build_ros_graph(rp_color_path, rp_depth_path)

    omni.timeline.get_timeline_interface().play()
    simulation_app.update()

    # 비디오 라이터
    os.makedirs(OUT_DIR, exist_ok=True)
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    rgb_path = os.path.join(OUT_DIR, "spot_cam_rgb.mp4")
    depth_path = os.path.join(OUT_DIR, "spot_cam_depth.mp4")
    vw_rgb = cv2.VideoWriter(rgb_path, fourcc, FPS, RES)
    vw_depth = cv2.VideoWriter(depth_path, fourcc, FPS, RES)

    print(f"\n[REC START] {int(SECONDS * FPS)} 프레임 → {OUT_DIR}")
    print(f"   rosbag(같은 폴더): ros2 bag record -o {OUT_DIR}/spot_bag "
          "/spot_cam/rgb /spot_cam/depth /spot_cam/camera_info /clock\n")

    n_frames = int(SECONDS * FPS)
    written = 0
    for i in range(n_frames):
        # 정책 보행(정지 명령) + 물리
        with torch.inference_mode():
            actions = policy(obs["policy"] if isinstance(obs, dict) else obs)
            obs, _, _, _, _ = env.step(actions)
        simulation_app.update()   # 렌더(annotator/ROS 갱신)

        rgb = rgb_an.get_data()
        depth = depth_an.get_data()
        if rgb is not None and getattr(rgb, "size", 0) > 0:
            arr = np.asarray(rgb)
            if arr.ndim == 3 and arr.shape[2] >= 3:
                bgr = cv2.cvtColor(arr[:, :, :3].astype(np.uint8), cv2.COLOR_RGB2BGR)
                if bgr.shape[:2] != (RES[1], RES[0]):
                    bgr = cv2.resize(bgr, RES)
                vw_rgb.write(bgr)
                written += 1
        if depth is not None and getattr(depth, "size", 0) > 0:
            d = np.asarray(depth, dtype=np.float32)
            d = np.nan_to_num(d, nan=DEPTH_FAR, posinf=DEPTH_FAR, neginf=DEPTH_FAR)
            d = np.clip(d, DEPTH_NEAR, DEPTH_FAR)
            dn = ((d - DEPTH_NEAR) / (DEPTH_FAR - DEPTH_NEAR) * 255.0).astype(np.uint8)
            dc = cv2.applyColorMap(255 - dn, cv2.COLORMAP_JET)   # 가까울수록 빨강
            if dc.shape[:2] != (RES[1], RES[0]):
                dc = cv2.resize(dc, RES)
            vw_depth.write(dc)
        if (i + 1) % FPS == 0:
            print(f"  ... {i + 1}/{n_frames} 프레임 (rgb 기록 {written})")

    vw_rgb.release()
    vw_depth.release()
    print(f"\n✅ 녹화 완료:")
    print(f"   RGB  : {rgb_path}")
    print(f"   Depth: {depth_path}")
    env.close()
    simulation_app.close()


if __name__ == "__main__":
    main()
