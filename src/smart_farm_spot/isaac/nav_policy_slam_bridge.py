#!/usr/bin/env python3
# Copyright (c) 2024, Smart Farm Patrol Project - 꼬마 로봇 두리
# SPDX-License-Identifier: Apache-2.0
"""
nav_policy_slam_bridge.py
=========================
강화학습 보행 정책(235차원) + 축사 맵 + RTX 라이다 + SLAM 용 통합 브릿지.

핵심:
  - IsaacLab RL env(Isaac-Velocity-Rough-SpotArm-Play-v0) **험지 terrain 유지**
    → height_scan(187) 유효 = 235차원 그대로. (평면화하면 48차원으로 죽음)
  - **마찰계수 영역 분리**: 전역 terrain 마찰 = 통로(기본 0.7),
    그 위에 배변 우리 저마찰 패치(기본 0.45) 콜라이더를 덮음.
  - 축사 USD(environment_final.usd) 로드 → 벽이 라이다/SLAM 피처.
  - RTX 라이다 → /scan, base_link→lidar_frame static TF.
  - map→odom 은 **발행 안 함**(SLAM_toolbox 가 발행). odom/tf(odom→base)/clock 만.

ROS 인터페이스(rclpy 없이 OmniGraph):
  구독 /cmd_vel  → base_velocity 명령 주입
  발행 /scan(LaserScan), /odom, /tf(odom→base_link), /clock

실행:
  source ~/dev_ws/venv/isaaclab/bin/activate
  cd ~/dev_ws/isaac_sim/IsaacLab
  ./isaaclab.sh -p ~/dev_ws/spot_ws/src/smart_farm_spot/isaac/nav_policy_slam_bridge.py
"""

import argparse
import math
import os

os.environ["ROS_DOMAIN_ID"] = os.environ.get("ROS_DOMAIN_ID", "153")

_ASSETS = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "assets")
TASK = "Isaac-Velocity-Rough-SpotArm-Play-v0"
POLICY = os.path.join(_ASSETS, "policy", "policy.pt")
ENV_USD = os.path.join(_ASSETS, "scene", "environment_final.usd")

# ── 마찰계수 (마찰존) ───────────────────────────────────────────────
CORRIDOR_FRICTION = float(os.environ.get("SF_CORRIDOR_FRICTION", "0.7"))  # 통로
PEN_FRICTION      = float(os.environ.get("SF_PEN_FRICTION", "0.45"))      # 배변 우리
#  우리 패치 영역: cx,cy,hx,hy  (H 코스 중앙 → 대각선 횡단 시 미끄러짐 시연)
_pen = os.environ.get("SF_PEN_REGION", "1.0,1.5,0.9,0.9").split(",")
PEN_CX, PEN_CY, PEN_HX, PEN_HY = (float(v) for v in _pen)

# ── 라이다 ──────────────────────────────────────────────────────────
LIDAR_MOUNT_Z = 0.35
LIDAR_CONFIG  = "Example_Rotary_2D"
TOPIC_SCAN    = "/scan"
FRAME_LIDAR   = "lidar_frame"

LOAD_BARN = os.environ.get("SF_BARN", "1") == "1"

parser = argparse.ArgumentParser()
parser.add_argument("--task", default=TASK)
parser.add_argument("--policy", default=POLICY)
parser.add_argument("--headless", action="store_true")
a = parser.parse_args()

from isaaclab.app import AppLauncher  # noqa: E402
app_launcher = AppLauncher(headless=a.headless)
simulation_app = app_launcher.app

import torch  # noqa: E402
import gymnasium as gym  # noqa: E402
import numpy as np  # noqa: E402
import omni.graph.core as og  # noqa: E402
import omni.kit.commands  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade  # noqa: E402
from isaacsim.core.utils.extensions import enable_extension  # noqa: E402
from isaacsim.core.utils.stage import add_reference_to_stage  # noqa: E402

import isaaclab_tasks  # noqa: F401,E402  (태스크 등록)
from isaaclab_tasks.utils import parse_env_cfg  # noqa: E402

NS_ROS2 = "isaacsim.ros2.bridge"
NS_CORE = "isaacsim.core.nodes"
GRAPH = "/NavPolicySlamBridge"


def _yaw_from_quat(qw, qx, qy, qz):
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def make_pen_patch(stage, cx, cy, hx, hy, friction, z_top, idx=0):
    """흙(배변 우리) 저마찰 패치: 얇은 콜라이더(저마찰 PhysicsMaterial)
    + 흙색(갈색) UsdPreviewSurface 시각재질 → 맵의 흙과 색 일치."""
    patch_path = f"/World/PenPatch_{idx}"
    cube = UsdGeom.Cube.Define(stage, patch_path)
    cube.GetSizeAttr().Set(2.0)   # 기본 2x2x2 → 스케일로 얇게
    xf = UsdGeom.Xformable(cube.GetPrim())
    xf.ClearXformOpOrder()
    thick = 0.04
    xf.AddTranslateOp().Set(Gf.Vec3d(cx, cy, z_top - thick / 2.0))
    xf.AddScaleOp().Set(Gf.Vec3f(hx, hy, thick / 2.0))
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())

    # 물리: 저마찰
    mat_path = f"/World/PenPhysMat_{idx}"
    mat = UsdShade.Material.Define(stage, mat_path)
    UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
    pm = UsdPhysics.MaterialAPI(mat.GetPrim())
    pm.CreateStaticFrictionAttr(friction)
    pm.CreateDynamicFrictionAttr(friction)
    pm.CreateRestitutionAttr(0.0)
    binding = UsdShade.MaterialBindingAPI.Apply(cube.GetPrim())
    binding.Bind(mat, bindingStrength=UsdShade.Tokens.strongerThanDescendants,
                 materialPurpose="physics")

    # 시각: 흙색(갈색) — 맵 흙과 색 일치
    vis_path = f"/World/PenVisMat_{idx}"
    vis = UsdShade.Material.Define(stage, vis_path)
    shader = UsdShade.Shader.Define(stage, f"{vis_path}/Surface")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
        Gf.Vec3f(0.42, 0.30, 0.18))     # 흙 갈색
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.95)
    vis.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    UsdShade.MaterialBindingAPI(cube.GetPrim()).Bind(vis)   # 기본 purpose(시각)
    return patch_path


def find_dirt_meshes(stage, barn_root, log=True):
    """축사 안 흙(모래/자갈) 바닥 메시의 월드 XY 영역 자동 탐지.
    이름에 sand/gravel/dirt/soil/pen 등이 들어간 Mesh 의 월드 bbox 중심·반경 반환.
    (barn translate 적용 후 호출해야 월드좌표 정확)."""
    regions = []
    root = stage.GetPrimAtPath(barn_root)
    if not root or not root.IsValid():
        return regions
    bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                                   [UsdGeom.Tokens.default_])
    # 우리(배변 구역)=Bedding_*_Zone 만 저마찰. Sand_*_Corridor(통로)는 고마찰(전역).
    kws = ("bedding", "_pen", "manure", "dirt", "soil", "mud", "흙")
    allmesh = []
    for p in Usd.PrimRange(root):
        if p.GetTypeName() != "Mesh":
            continue
        nm = p.GetName()
        allmesh.append(nm)
        if any(k in nm.lower() for k in kws):
            try:
                r = bbox_cache.ComputeWorldBound(p).ComputeAlignedRange()
                mn, mx = r.GetMin(), r.GetMax()
                cx, cy = (mn[0] + mx[0]) / 2.0, (mn[1] + mx[1]) / 2.0
                hx = max(0.3, (mx[0] - mn[0]) / 2.0)
                hy = max(0.3, (mx[1] - mn[1]) / 2.0)
                regions.append((nm, cx, cy, hx, hy, float(mx[2])))
            except Exception:
                pass
    if log:
        print(f"  [축사 메시 {len(allmesh)}개] " + ", ".join(allmesh[:50]))
    return regions


def build_ros_graph(lidar_rp_path):
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
                ("TfLidar",  f"{NS_ROS2}.ROS2PublishRawTransformTree"),
                ("LidarPub", f"{NS_ROS2}.ROS2RtxLidarHelper"),
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
                # base_link → lidar_frame (static, 라이다 마운트 오프셋)
                ("TfLidar.inputs:topicName", "/tf_static"),
                ("TfLidar.inputs:parentFrameId", "base_link"),
                ("TfLidar.inputs:childFrameId", FRAME_LIDAR),
                ("TfLidar.inputs:translation", [0.0, 0.0, LIDAR_MOUNT_Z]),
                ("TfLidar.inputs:staticPublisher", True),
                # RTX 라이다 → /scan
                ("LidarPub.inputs:renderProductPath", lidar_rp_path or ""),
                ("LidarPub.inputs:topicName", TOPIC_SCAN),
                ("LidarPub.inputs:frameId", FRAME_LIDAR),
                ("LidarPub.inputs:type", "laser_scan"),
            ],
            keys.CONNECT: [
                ("Tick.outputs:tick", "ClockPub.inputs:execIn"),
                ("Tick.outputs:tick", "SubTwist.inputs:execIn"),
                ("Tick.outputs:tick", "OdomPub.inputs:execIn"),
                ("Tick.outputs:tick", "TfOdom.inputs:execIn"),
                ("Tick.outputs:tick", "TfLidar.inputs:execIn"),
                ("Tick.outputs:tick", "LidarPub.inputs:execIn"),
                ("SimTime.outputs:simulationTime", "ClockPub.inputs:timeStamp"),
                ("SimTime.outputs:simulationTime", "OdomPub.inputs:timeStamp"),
                ("SimTime.outputs:simulationTime", "TfOdom.inputs:timeStamp"),
                ("SimTime.outputs:simulationTime", "TfLidar.inputs:timeStamp"),
            ],
        },
    )
    A = og.Controller.attribute
    return {
        "sub_lin": A(f"{GRAPH}/SubTwist.outputs:linearVelocity"),
        "sub_ang": A(f"{GRAPH}/SubTwist.outputs:angularVelocity"),
        "od_pos": A(f"{GRAPH}/OdomPub.inputs:position"),
        "od_ori": A(f"{GRAPH}/OdomPub.inputs:orientation"),
        "od_lin": A(f"{GRAPH}/OdomPub.inputs:linearVelocity"),
        "od_ang": A(f"{GRAPH}/OdomPub.inputs:angularVelocity"),
        "tf_t": A(f"{GRAPH}/TfOdom.inputs:translation"),
        "tf_r": A(f"{GRAPH}/TfOdom.inputs:rotation"),
    }


def main():
    print("=" * 60)
    print(" RL 보행정책(235차원) + 축사맵 + 라이다 + SLAM 브릿지")
    print(f"   task={a.task}  domain={os.getenv('ROS_DOMAIN_ID')}")
    print(f"   마찰: 통로={CORRIDOR_FRICTION} / 우리={PEN_FRICTION} "
          f"@ ({PEN_CX},{PEN_CY}) ±({PEN_HX},{PEN_HY})")
    print("=" * 60)

    # ── env 생성 (1 env, GUI) — 험지 terrain 유지(235차원) ───────────
    env_cfg = parse_env_cfg(a.task, device="cuda:0", num_envs=1)
    # ── 자동 리셋(에피소드 종료) 끔 ──────────────────────────────────
    #  Play env 는 max_episode_length / 넘어짐마다 로봇을 다른 terrain 타일로
    #  텔레포트(리셋)시킨다 → odom 25m 점프 → SLAM/Nav2 깨짐. 지속 로봇으로
    #  쓰려면 모든 termination 제거.
    try:
        for _name in [n for n in vars(env_cfg.terminations) if not n.startswith("_")]:
            setattr(env_cfg.terminations, _name, None)
        print("  ✅ 자동 리셋(termination) 끔 — 로봇 지속(텔레포트 방지)")
    except Exception as e:
        print(f"  ⚠️ termination 끄기 실패: {e}")
    # 축사 정렬을 위해 평면 물리바닥(z=0) 사용 → 로봇이 월드 원점에 스폰되어
    #  축사(원점 배치)와 정확히 정렬. (험지 타일은 (15,-16)에 기울어져 축사와 어긋남)
    #  관측은 여전히 235차원(height_scan 187). 평탄하지만 마찰존으로 우리 미끄러움 시연.
    if os.environ.get("SF_ROUGH", "0") == "1":
        print("  ✅ 맵: 학습 험지(rough terrain) — 235차원 height_scan 의미↑ (축사 어긋날 수 있음)")
    else:
        env_cfg.scene.terrain.terrain_type = "plane"
        env_cfg.scene.terrain.terrain_generator = None
        try:
            env_cfg.curriculum.terrain_levels = None
        except Exception:
            pass
        print("  ✅ 맵: 평면 물리바닥(z=0) — 축사 정렬용(로봇 원점 스폰)")
    # 전역 terrain 마찰 = 통로 마찰
    try:
        env_cfg.scene.terrain.physics_material.static_friction = CORRIDOR_FRICTION
        env_cfg.scene.terrain.physics_material.dynamic_friction = CORRIDOR_FRICTION
        print(f"  ✅ 전역 terrain 마찰 = 통로 {CORRIDOR_FRICTION}")
    except Exception as e:
        print(f"  ⚠️ terrain 마찰 설정 실패: {e}")

    env = gym.make(a.task, cfg=env_cfg)
    obs, _ = env.reset()
    device = env.unwrapped.device

    stage = omni.usd.get_context().get_stage()
    robot = env.unwrapped.scene["robot"]
    robot_prim = robot.root_physx_view.prim_paths[0]
    print(f"  ✅ 로봇 prim: {robot_prim}")

    # ── 로봇 실제 스폰 위치 (RL 험지 env 는 월드 원점이 아닌 terrain 타일에 스폰) ──
    #  축사/우리 패치를 이 위치 기준으로 배치해야 라이다가 보고 마찰존이 맞음.
    _rs = robot.data.root_state_w[0]
    SX, SY, SZ = float(_rs[0]), float(_rs[1]), float(_rs[2])
    if not (math.isfinite(SX) and math.isfinite(SY) and math.isfinite(SZ)):
        SX = SY = 0.0; SZ = 0.6
    if os.environ.get("SF_ROUGH", "0") == "1":
        FLOOR_Z = SZ - 0.5      # 험지: 베이스에서 ~0.5 아래
        z_top = SZ - 0.45
    else:
        FLOOR_Z = 0.0           # 평면(z=0): 축사 바닥을 물리 평면과 일치
        z_top = 0.02            # 흙 패치를 평면 바로 위(발이 닿게)
    print(f"  ✅ 로봇 스폰 월드좌표: ({SX:.2f},{SY:.2f},{SZ:.2f}) floor_z={FLOOR_Z:.2f}")

    # ── 축사 USD (시각 + 라이다 피처) — 스폰 위치로 이동 ─────────────
    barn_loaded = False
    if LOAD_BARN and os.path.exists(ENV_USD):
        try:
            add_reference_to_stage(usd_path=ENV_USD, prim_path="/World/Barn")
            _bx = UsdGeom.Xformable(stage.GetPrimAtPath("/World/Barn"))
            _bx.ClearXformOpOrder()
            _bx.AddTranslateOp().Set(Gf.Vec3d(SX, SY, FLOOR_Z))
            simulation_app.update()   # bbox 계산 위해 트랜스폼 반영
            barn_loaded = True
            print(f"  ✅ 축사 환경 로드: /World/Barn @ world({SX:.2f},{SY:.2f},{FLOOR_Z:.2f})")
        except Exception as e:
            print(f"  ⚠️ 축사 로드 실패: {e}")
    else:
        print(f"  ⚠️ 축사 USD 생략(SF_BARN={LOAD_BARN}, exists={os.path.exists(ENV_USD)})")

    # ── 흙(모래/자갈) 우리 바닥 자동 탐지 → 갈색 저마찰 패치 ──────────
    dirt = find_dirt_meshes(stage, "/World/Barn") if barn_loaded else []
    if dirt:
        for i, (nm, cx, cy, hx, hy, ztop) in enumerate(dirt):
            try:
                # 패치 z 는 로봇이 걷는 바닥(z_top) 기준 — 메시 z 가 아니라.
                make_pen_patch(stage, cx, cy, hx, hy, PEN_FRICTION, z_top, idx=i)
                print(f"  ✅ 흙 우리 저마찰 패치[{i}] '{nm}' "
                      f"@ ({cx:.2f},{cy:.2f}) ±({hx:.2f},{hy:.2f}) μ={PEN_FRICTION}")
            except Exception as e:
                print(f"  ⚠️ 패치[{i}] 실패: {e}")
    else:
        # 자동탐지 실패 → 수동 영역(SF_PEN_REGION) 폴백
        try:
            make_pen_patch(stage, SX + PEN_CX, SY + PEN_CY, PEN_HX, PEN_HY,
                           PEN_FRICTION, z_top, idx=0)
            print(f"  ⚠️ 흙 메시 자동탐지 실패 → 수동 패치 @ "
                  f"world({SX+PEN_CX:.2f},{SY+PEN_CY:.2f}) μ={PEN_FRICTION}")
        except Exception as e:
            print(f"  ⚠️ 수동 패치 실패: {e}")

    # ── 로봇 스폰 높이 살짝 올리기 (바닥 박힘 방지, 공중에서 안착) ───
    LIFT_Z = float(os.environ.get("SF_LIFT_Z", "0.12"))
    if LIFT_Z:
        try:
            _r2 = robot.data.root_state_w.clone()
            _r2[:, 2] += LIFT_Z
            robot.write_root_pose_to_sim(_r2[:, :7])
            robot.write_root_velocity_to_sim(torch.zeros_like(_r2[:, 7:13]))
            print(f"  ✅ 로봇 스폰 +{LIFT_Z:.2f}m (공중 안착)")
        except Exception as e:
            print(f"  ⚠️ 스폰 높이 올리기 실패: {e}")

    # ── RTX 라이다 (로봇 베이스에 마운트) ───────────────────────────
    lidar_mount = f"{robot_prim}/Lidar360"
    _lm = UsdGeom.Xform.Define(stage, lidar_mount)
    _lx = UsdGeom.Xformable(_lm.GetPrim())
    _lx.ClearXformOpOrder()
    _lx.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, LIDAR_MOUNT_Z))
    lidar_prim = None
    try:
        _, _lidar = omni.kit.commands.execute(
            "IsaacSensorCreateRtxLidar", path="/lidar_sensor",
            parent=lidar_mount, config=LIDAR_CONFIG)
        lidar_prim = f"{lidar_mount}/lidar_sensor"
        print(f"  ✅ RTX Lidar: {lidar_prim} (config={LIDAR_CONFIG})")
    except Exception as e:
        print(f"  ❌ Lidar 생성 실패: {e}")
    simulation_app.update()

    import omni.replicator.core as rep
    lidar_rp_path = None
    if lidar_prim is not None:
        try:
            _lidar_rp = rep.create.render_product(lidar_prim, [1, 1])
            lidar_rp_path = _lidar_rp.path if hasattr(_lidar_rp, "path") else str(_lidar_rp)
            print(f"  ✅ Lidar 렌더프로덕트: {lidar_rp_path}")
        except Exception as e:
            print(f"  ❌ Lidar 렌더프로덕트 실패: {e}")

    # ── 정책 로드 (TorchScript) ──────────────────────────────────────
    policy = torch.jit.load(a.policy, map_location=device)
    policy.eval()
    print(f"  ✅ 정책 로드: {a.policy} (device={device})")

    # ── ROS2 브릿지 ──────────────────────────────────────────────────
    enable_extension("isaacsim.ros2.bridge")
    simulation_app.update()
    attrs = build_ros_graph(lidar_rp_path)
    print("  ✅ OmniGraph ROS2: /clock /cmd_vel(sub) /odom /tf(odom→base) "
          "/tf_static(base→lidar) /scan")
    print("     (map→odom 은 SLAM_toolbox 가 발행 — 여기선 안 함)")

    cmd_term = env.unwrapped.command_manager.get_term("base_velocity")
    try:
        cmd_term.cfg.resampling_time_range = (1.0e9, 1.0e9)
    except Exception:
        pass
    # 헤딩 추종 각속도 강제 끔 — 리셋 때 뽑힌 랜덤 헤딩을 매 스텝 추종해
    #  wz=0 을 덮어쓰며 "게걸음"(곡선 드리프트)을 만들던 원인.
    try:
        cmd_term.cfg.heading_command = False
        cmd_term.heading_target[:] = 0.0
    except Exception:
        pass
    # 리셋 때 뽑힌 잔류 명령 0 으로 초기화(정지에서 시작)
    try:
        cmd_term.vel_command_b[:] = 0.0
    except Exception:
        pass
    x0 = y0 = None

    def policy_obs(o):
        t = o["policy"] if isinstance(o, dict) else o
        return t.to(device)

    print("\n[실행] RL 보행 + 라이다 + SLAM. 시스템 ROS2(같은 도메인)에서:")
    print("   ros2 launch smart_farm_spot spot_slam.launch.py")
    print("   ros2 launch smart_farm_spot nav2_slam.launch.py")
    print("   python3 h_drive.py --waypoints config/waypoints_diag.yaml\n")

    # OmniGraph ROS 퍼블리셔(OnPlaybackTick)가 틱하도록 타임라인 재생 보장
    try:
        omni.timeline.get_timeline_interface().play()
        print("  ✅ 타임라인 play() — ROS 퍼블리셔 활성")
    except Exception as e:
        print(f"  ⚠️ timeline play 실패: {e}")

    step = 0
    while simulation_app.is_running():
        lin = og.Controller.get(attrs["sub_lin"])
        ang = og.Controller.get(attrs["sub_ang"])
        vx = float(lin[0]) if lin is not None else 0.0
        wz = float(ang[2]) if ang is not None else 0.0
        cmd_term.vel_command_b[:, 0] = vx
        cmd_term.vel_command_b[:, 1] = 0.0
        cmd_term.vel_command_b[:, 2] = wz

        with torch.inference_mode():
            actions = policy(policy_obs(obs))
            obs, _, _, _, _ = env.step(actions)

        rs = robot.data.root_state_w[0]
        x, y = float(rs[0]), float(rs[1])
        if x0 is None:
            x0, y0 = x, y
        ox, oy = x - x0, y - y0
        qw, qx, qy, qz = float(rs[3]), float(rs[4]), float(rs[5]), float(rs[6])
        yaw = _yaw_from_quat(qw, qx, qy, qz)
        oqw, oqx, oqy, oqz = math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)
        og.Controller.set(attrs["od_pos"], [ox, oy, 0.0])
        og.Controller.set(attrs["od_ori"], [oqx, oqy, oqz, oqw])
        og.Controller.set(attrs["od_lin"], [vx, 0.0, 0.0])
        og.Controller.set(attrs["od_ang"], [0.0, 0.0, wz])
        og.Controller.set(attrs["tf_t"], [ox, oy, 0.0])
        og.Controller.set(attrs["tf_r"], [oqx, oqy, oqz, oqw])

        step += 1
        if step % 120 == 0:
            print(f"  ... {step}스텝 | cmd=({vx:.2f},{wz:.2f}) "
                  f"pose=({x:.2f},{y:.2f},{math.degrees(yaw):.0f}°)")

    env.close()
    simulation_app.close()


if __name__ == "__main__":
    main()
