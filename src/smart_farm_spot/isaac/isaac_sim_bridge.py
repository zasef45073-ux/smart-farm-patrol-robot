#!/usr/bin/env python3
# Copyright (c) 2024, Smart Farm Patrol Project - 꼬마 로봇 두리
# SPDX-License-Identifier: Apache-2.0
"""
isaac_sim_bridge.py
===================
Spot + Arm 통합 Isaac Sim ↔ ROS 2 브릿지 (Isaac Sim 5.1).

기존에 분리돼 있던 두 브릿지를 하나로 병합:
  - isaac_ros2_bridge.py  (센서 OmniGraph: 카메라/열화상/LiDAR/IMU/객체인식)
  - isaac_nav_bridge.py   (Nav2 폐루프: /cmd_vel 수신 → 구동, /odom + TF 발행)

⚠️  이 파일은 ROS 2 노드가 아니라 Isaac Sim 안에서 실행됩니다.
    (SimulationApp 필요 → `ros2 run` 불가, `python.sh`로 실행)

    실행:
      ~/isaacsim/python.sh \
        $(ros2 pkg prefix smart_farm_spot)/share/smart_farm_spot/isaac/isaac_sim_bridge.py \
        --usd /path/to/barn_scene.usd \
        --robot-prim /World/Spot \
        --mode kinematic

[발행 토픽]
  /clock                          rosgraph_msgs/Clock          시뮬 시간 동기화
  /spot/arm/rgb/image_raw         sensor_msgs/Image            RGB 영상
  /spot/arm/rgb/camera_info       sensor_msgs/CameraInfo       RGB 내부 파라미터
  /spot/arm/detections            vision_msgs/Detection2DArray ★객체 인식(정답 bbox)★
  /spot/arm/thermal/image_raw     sensor_msgs/Image            열화상(모사) 영상
  /spot/scan                      sensor_msgs/LaserScan        LiDAR (Nav2 충돌 회피)
  /spot/imu/data                  sensor_msgs/Imu              IMU
  /odom                           nav_msgs/Odometry            주행 거리계 (Nav2 AMCL)
  /tf  (odom→base_link + 센서)    tf2_msgs/TFMessage           좌표 변환

[구독 토픽]
  /cmd_vel                        geometry_msgs/Twist          Nav2 속도 명령

[TF 소유권 - 병합 시 충돌 방지]
  센서 OmniGraph의 TF 발행은 끄고(SENSOR_GRAPH_TF=False),
  TF는 Nav 브릿지가 전담:
    odom→base_link (동적) + base_link→센서 프레임(static).
  → 센서 frame_id(lidar_frame/arm_rgb_frame/...)는 STATIC_TF로 보장됨.

[구동 모드]
  kinematic : cmd_vel을 베이스 속도로 직접 적용 (RL 정책 없이 Nav2 시연)
  policy    : 학습된 보행 RL 정책으로 전달 (_apply_policy 에 연결)
"""

import argparse
import math
import os

# ══════════════════════════════════════════════════════════════════════
# 0. 인자 파싱 → SimulationApp 먼저 생성 (다른 import보다 반드시 먼저)
# ══════════════════════════════════════════════════════════════════════
parser = argparse.ArgumentParser(description="Spot 통합 Isaac Sim ↔ ROS 2 브릿지")
parser.add_argument("--usd", type=str, default=None,
                    help="로드할 USD 씬 경로 (소+축사 통합 씬 권장)")
parser.add_argument("--robot-usd", type=str, default=None,
                    help="로봇 USD 경로 (씬에 로봇이 없을 때 단독 스폰)")
parser.add_argument("--robot-prim", type=str, default="/World/Spot",
                    help="로봇 articulation prim 경로")
parser.add_argument("--mode", choices=["kinematic", "policy"], default="kinematic",
                    help="kinematic: cmd_vel 직접 적용 / policy: 보행 RL 정책")
parser.add_argument("--gui", action="store_true", help="GUI 표시 (기본: 헤드리스)")
parser.add_argument("--no-sensors", action="store_true",
                    help="센서 OmniGraph 브릿지 비활성화 (Nav만)")
parser.add_argument("--no-semantics", action="store_true",
                    help="객체 인식 자동 시맨틱 라벨링 비활성화")
args = parser.parse_args()

# IsaacLab AppLauncher 로 앱 생성 (bare SimulationApp 은 이 환경에서 omni.kit.usd 에러).
from isaaclab.app import AppLauncher  # noqa: E402
simulation_app = AppLauncher(headless=not args.gui).app

# ── 앱 생성 후 나머지 import ─────────────────────────────────────────
import numpy as np  # noqa: E402
import omni.usd  # noqa: E402
import carb  # noqa: E402

from isaacsim.core.utils.extensions import enable_extension  # noqa: E402
enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.utils.stage import add_reference_to_stage, get_current_stage  # noqa: E402
from isaacsim.core.prims import SingleArticulation  # noqa: E402
import isaacsim.core.utils.prims as prim_utils  # noqa: E402

import omni.graph.core as og  # noqa: E402
import omni.replicator.core as rep  # noqa: E402

import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import QoSProfile, ReliabilityPolicy  # noqa: E402
from geometry_msgs.msg import Twist, TransformStamped  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster  # noqa: E402


# ══════════════════════════════════════════════════════════════════════
# 설정 상수
# ══════════════════════════════════════════════════════════════════════

# ── 해상도 (검출용 RGB는 HD, 열화상은 작게) ──────────────────────────
RGB_RESOLUTION     = (1280, 720)
THERMAL_RESOLUTION = (640, 512)

# 열화상 카메라 비전 준비중 → 통합에서 제외. 비전팀 작업 완료 시 True 로.
ENABLE_THERMAL = False

# ── ROS 2 토픽 ────────────────────────────────────────────────────────
TOPIC_RGB        = "/spot/arm/rgb/image_raw"
TOPIC_RGB_INFO   = "/spot/arm/rgb/camera_info"
TOPIC_DETECTION  = "/spot/arm/detections"
TOPIC_THERMAL    = "/spot/arm/thermal/image_raw"
TOPIC_SCAN       = "/spot/scan"
TOPIC_IMU        = "/spot/imu/data"

# ── 프레임 ID (Nav 브릿지 static TF와 반드시 일치) ───────────────────
FRAME_RGB     = "arm_rgb_frame"
FRAME_THERMAL = "arm_thermal_frame"
FRAME_LIDAR   = "lidar_frame"
FRAME_IMU     = "imu_frame"

GRAPH_PATH = "/SpotSensorBridge"

# 센서 OmniGraph가 TF를 발행할지 여부.
# 병합 환경에서는 Nav 브릿지가 TF를 전담하므로 False 권장(중복/충돌 방지).
SENSOR_GRAPH_TF = False

# ── Isaac Sim 익스텐션 네임스페이스 (5.1: isaacsim.* / 구버전: omni.isaac.*) ──
USE_LEGACY = False
if USE_LEGACY:
    NS_ROS2, NS_CORE, NS_SENS = "omni.isaac.ros2_bridge", "omni.isaac.core_nodes", "omni.isaac.sensor"
else:
    NS_ROS2, NS_CORE, NS_SENS = "isaacsim.ros2.bridge", "isaacsim.core.nodes", "isaacsim.sensors.physics"

# base_link 기준 센서 static TF (child, x, y, z) — Nav 브릿지가 발행
STATIC_TF = [
    ("lidar_frame",       0.0,  0.0, 0.30),
    ("imu_frame",         0.0,  0.0, 0.00),
    ("arm_rgb_frame",     0.05, 0.0, 0.55),
    ("arm_thermal_frame", -0.05, 0.0, 0.55),
]


# ══════════════════════════════════════════════════════════════════════
# 1. 센서 prim 경로 (robot-prim 기준 동적 구성)
# ══════════════════════════════════════════════════════════════════════
def sensor_prims(robot_prim: str) -> dict:
    """robot-prim 접두사 기준 센서/링크 prim 경로 딕셔너리."""
    return {
        "rgb":     f"{robot_prim}/arm0_link_fngr/RGBCamera",
        "thermal": f"{robot_prim}/arm0_link_fngr/ThermalCamera",
        "lidar":   f"{robot_prim}/base/Lidar360/lidar_sensor",
        "imu":     f"{robot_prim}/base/IMU",
        "base":    f"{robot_prim}/base",
        "fngr":    f"{robot_prim}/arm0_link_fngr",
    }


# ══════════════════════════════════════════════════════════════════════
# 2. 센서 OmniGraph 브릿지 (카메라/열화상/LiDAR/IMU/객체인식 + Clock)
# ══════════════════════════════════════════════════════════════════════
def setup_sensor_bridge(prims: dict, auto_semantics: bool = True):
    """모든 센서를 ROS 2로 발행하는 OmniGraph ActionGraph 생성."""
    print("=" * 60)
    print(" [센서] Isaac Sim 센서 OmniGraph 브릿지 셋업")
    print("=" * 60)

    stage = omni.usd.get_context().get_stage()
    for key in ("rgb", "thermal", "lidar", "imu"):
        prim = stage.GetPrimAtPath(prims[key])
        mark = "✅" if (prim and prim.IsValid()) else "❌ 없음"
        print(f"  센서 확인: {prims[key]}  {mark}")

    # ── 객체 인식용 시맨틱 라벨링 ────────────────────────────────────
    if auto_semantics:
        print("\n  [0] 객체 인식 시맨틱 라벨링...")
        try:
            from setup_semantics import auto_tag_by_keyword
            auto_tag_by_keyword("/World")
        except Exception as e:
            print(f"    ⚠️  자동 라벨링 건너뜀: {e}")
            print("       소 에셋 로드 후 setup_semantics.tag_semantics()로 수동 지정하세요.")

    # ── 렌더 프로덕트 생성 (카메라 → GPU 렌더 출력) ─────────────────
    rgb_rp = rep.create.render_product(prims["rgb"], RGB_RESOLUTION)
    rgb_path = rgb_rp.path if hasattr(rgb_rp, "path") else str(rgb_rp)
    print(f"    ✅ RGB     렌더프로덕트: {rgb_path}  {RGB_RESOLUTION}")
    if ENABLE_THERMAL:
        thermal_rp = rep.create.render_product(prims["thermal"], THERMAL_RESOLUTION)
        thermal_path = thermal_rp.path if hasattr(thermal_rp, "path") else str(thermal_rp)
        print(f"    ✅ Thermal 렌더프로덕트: {thermal_path}  {THERMAL_RESOLUTION}")

    # ── OmniGraph 구성 ──────────────────────────────────────────────
    keys = og.Controller.Keys
    create_nodes = [
        ("Tick",       "omni.graph.action.OnPlaybackTick"),
        ("SimTime",    f"{NS_CORE}.IsaacReadSimulationTime"),
        ("ClockPub",   f"{NS_ROS2}.ROS2PublishClock"),
        ("RgbPub",     f"{NS_ROS2}.ROS2CameraHelper"),
        ("DetectPub",  f"{NS_ROS2}.ROS2CameraHelper"),
        ("LidarPub",   f"{NS_ROS2}.ROS2RtxLidarHelper"),
        ("ImuRead",    f"{NS_SENS}.IsaacReadIMU"),
        ("ImuPub",     f"{NS_ROS2}.ROS2PublishImu"),
    ]
    set_values = [
        ("ClockPub.inputs:topicName", "/clock"),

        ("RgbPub.inputs:renderProductPath", rgb_path),
        ("RgbPub.inputs:type",       "rgb"),
        ("RgbPub.inputs:topicName",  TOPIC_RGB),
        ("RgbPub.inputs:frameId",    FRAME_RGB),

        # 같은 RGB 렌더프로덕트, bbox_2d_tight → Detection2DArray
        ("DetectPub.inputs:renderProductPath", rgb_path),
        ("DetectPub.inputs:type",      "bbox_2d_tight"),
        ("DetectPub.inputs:topicName", TOPIC_DETECTION),
        ("DetectPub.inputs:frameId",   FRAME_RGB),

        ("LidarPub.inputs:topicName", TOPIC_SCAN),
        ("LidarPub.inputs:frameId",   FRAME_LIDAR),
        ("LidarPub.inputs:type",      "laser_scan"),

        ("ImuPub.inputs:topicName",   TOPIC_IMU),
        ("ImuPub.inputs:frameId",     FRAME_IMU),
    ]
    connect = [
        ("Tick.outputs:tick", "ClockPub.inputs:execIn"),
        ("Tick.outputs:tick", "RgbPub.inputs:execIn"),
        ("Tick.outputs:tick", "DetectPub.inputs:execIn"),
        ("Tick.outputs:tick", "LidarPub.inputs:execIn"),
        ("Tick.outputs:tick", "ImuRead.inputs:execIn"),
        ("SimTime.outputs:simulationTime", "ClockPub.inputs:timeStamp"),
        ("ImuRead.outputs:execOut",     "ImuPub.inputs:execIn"),
        ("ImuRead.outputs:linAcc",      "ImuPub.inputs:linearAcceleration"),
        ("ImuRead.outputs:angVel",      "ImuPub.inputs:angularVelocity"),
        ("ImuRead.outputs:orientation", "ImuPub.inputs:orientation"),
    ]

    # ── 열화상 카메라 (선택) — 비전 준비중이라 기본 비활성 ─────────
    if ENABLE_THERMAL:
        create_nodes.append(("ThermalPub", f"{NS_ROS2}.ROS2CameraHelper"))
        set_values += [
            ("ThermalPub.inputs:renderProductPath", thermal_path),
            ("ThermalPub.inputs:type",      "rgb"),
            ("ThermalPub.inputs:topicName", TOPIC_THERMAL),
            ("ThermalPub.inputs:frameId",   FRAME_THERMAL),
        ]
        connect.append(("Tick.outputs:tick", "ThermalPub.inputs:execIn"))

    # ── 센서 TF (선택) — 병합 시 기본 비활성 (Nav 브릿지가 TF 전담) ──
    if SENSOR_GRAPH_TF:
        create_nodes.append(("TfPub", f"{NS_ROS2}.ROS2PublishTransformTree"))
        set_values += [
            ("TfPub.inputs:topicName",   "/tf"),
            ("TfPub.inputs:targetPrims", [
                prims["base"], prims["fngr"], prims["lidar"],
                prims["imu"], prims["rgb"], prims["thermal"],
            ]),
        ]
        connect += [
            ("Tick.outputs:tick", "TfPub.inputs:execIn"),
            ("SimTime.outputs:simulationTime", "TfPub.inputs:timeStamp"),
        ]

    og.Controller.edit(
        {"graph_path": GRAPH_PATH, "evaluator_name": "execution"},
        {keys.CREATE_NODES: create_nodes,
         keys.SET_VALUES: set_values,
         keys.CONNECT: connect},
    )

    # ── 관계(relationship): IMU 읽기 노드의 대상 prim ──────────────
    imu_read = stage.GetPrimAtPath(f"{GRAPH_PATH}/ImuRead")
    rel = imu_read.GetRelationship("inputs:imuPrim")
    if not rel:
        rel = imu_read.CreateRelationship("inputs:imuPrim")
    rel.SetTargets([prims["imu"]])

    print("    ✅ Clock    → /clock")
    print(f"    ✅ RGB      → {TOPIC_RGB}")
    print(f"    ✅ 객체인식 → {TOPIC_DETECTION}  (Detection2DArray)")
    print(f"    {'✅ Thermal  → ' + TOPIC_THERMAL if ENABLE_THERMAL else '⏸  Thermal  → 비전 준비중(제외)'}")
    print(f"    ✅ LiDAR    → {TOPIC_SCAN}")
    print(f"    ✅ IMU      → {TOPIC_IMU}")
    print(f"    ✅ 센서 TF  → {'/tf (센서 그래프)' if SENSOR_GRAPH_TF else 'Nav 브릿지가 전담'}")


# ══════════════════════════════════════════════════════════════════════
# 직립/구동 보정 상수 + 쿼터니언 유틸
# ══════════════════════════════════════════════════════════════════════
# spot_with_arm.usd 는 그냥 두면 등으로 뒤집힘 → X축 180° 로 직립.
BASE_ROT_WXYZ   = (0.0, 1.0, 0.0, 0.0)
FACE_YAW_OFFSET = 180.0   # X180 보정으로 전방축 뒤집힘 → 진행방향 정렬
SPAWN_Z         = 0.65    # 베이스 높이(미끄러짐 유지)
SIM_DT          = 1.0 / 60.0


def _quat_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw*bw - ax*bx - ay*by - az*bz,
            aw*bx + ax*bw + ay*bz - az*by,
            aw*by - ax*bz + ay*bw + az*bx,
            aw*bz + ax*by - ay*bx + az*bw)


def _yaw_quat(yaw):
    """순수 Z축 yaw 쿼터니언 (w,x,y,z) — ROS odom 표준(z-up)."""
    h = yaw / 2.0
    return (math.cos(h), 0.0, 0.0, math.sin(h))


def _visual_quat(yaw):
    """직립 보정 + 전방 정렬 + yaw 를 합친 시각용 자세."""
    return _quat_mul(_yaw_quat(yaw + math.radians(FACE_YAW_OFFSET)), BASE_ROT_WXYZ)


# ══════════════════════════════════════════════════════════════════════
# 3. Nav 브릿지 ROS 2 노드 (/cmd_vel 수신 → 구동, /odom + TF 발행)
#    내부에 평면 상태(x,y,yaw)를 유지 → 시각적 뒤집힘과 ROS 프레임 분리.
# ══════════════════════════════════════════════════════════════════════
class IsaacNavBridge(Node):
    def __init__(self, articulation, mode, start_xy=(0.0, 0.0), start_yaw=0.0):
        super().__init__("isaac_nav_bridge")
        self.art = articulation
        self.mode = mode
        self.cmd_vx = 0.0
        self.cmd_wz = 0.0
        # 내부 평면 상태 (ROS odom 의 진실값)
        self.x, self.y = float(start_xy[0]), float(start_xy[1])
        self.yaw = float(start_yaw)
        # 초기 직립 포즈 적용
        self.art.set_world_pose(position=np.array([self.x, self.y, SPAWN_Z]),
                                orientation=np.array(_visual_quat(self.yaw)))

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE

        self.create_subscription(Twist, "/cmd_vel", self._cb_cmd_vel, qos)
        self.odom_pub = self.create_publisher(Odometry, "/odom", qos)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self._publish_static_tf()
        self.get_logger().info(f"Nav 브릿지 시작 (모드: {mode})")

    def _cb_cmd_vel(self, msg):
        self.cmd_vx = msg.linear.x
        self.cmd_wz = msg.angular.z

    def _publish_static_tf(self):
        transforms = []
        for child, x, y, z in STATIC_TF:
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = "base_link"
            t.child_frame_id = child
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.translation.z = z
            t.transform.rotation.w = 1.0
            transforms.append(t)
        self.static_broadcaster.sendTransform(transforms)
        self.get_logger().info(f"센서 static TF {len(transforms)}개 발행")

    def update(self, dt=SIM_DT):
        # ── cmd_vel 적분 (단순 차동구동 모델) ──
        self.yaw = math.atan2(math.sin(self.yaw + self.cmd_wz * dt),
                              math.cos(self.yaw + self.cmd_wz * dt))
        # 실제 위치는 베이스에서 읽어 적분 오차 누적 방지
        pos, _ = self.art.get_world_pose()
        self.x, self.y = float(pos[0]), float(pos[1])

        # ── 구동: 진행방향 직진(z 고정) + 시각 직립/정렬 ──
        self.art.set_world_pose(position=np.array([self.x, self.y, SPAWN_Z]),
                                orientation=np.array(_visual_quat(self.yaw)))
        vx_w = self.cmd_vx * math.cos(self.yaw)
        vy_w = self.cmd_vx * math.sin(self.yaw)
        self.art.set_linear_velocity(np.array([vx_w, vy_w, 0.0]))
        self.art.set_angular_velocity(np.zeros(3))

        self._publish_odom()

    def _publish_odom(self):
        stamp = self.get_clock().now().to_msg()
        qw, qx, qy, qz = _yaw_quat(self.yaw)   # 깨끗한 z-up ROS 자세

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.w = qw
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        self.tf_broadcaster.sendTransform(t)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.w = qw
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.twist.twist.linear.x = self.cmd_vx
        odom.twist.twist.angular.z = self.cmd_wz
        self.odom_pub.publish(odom)


def _quat_to_yaw(quat):
    w, x, y, z = float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3])
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny, cosy)


# ══════════════════════════════════════════════════════════════════════
# 4. 메인 — 씬 로드 → 센서 브릿지 + Nav 브릿지 → 스텝 루프
# ══════════════════════════════════════════════════════════════════════
def main():
    print("=" * 60)
    print(" Spot 통합 Isaac Sim ↔ ROS 2 브릿지")
    print(f"   모드: {args.mode} | 센서: {not args.no_sensors} | GUI: {args.gui}")
    print("=" * 60)

    world = World(stage_units_in_meters=1.0)

    # ── 씬 로드 ───────────────────────────────────────────────────
    if args.usd and os.path.exists(args.usd):
        print(f"[로드] 씬 USD: {args.usd}")
        omni.usd.get_context().open_stage(args.usd)
        simulation_app.update()
    else:
        print("[로드] 기본 지면 + 로봇 단독 스폰")
        world.scene.add_default_ground_plane()
        if args.robot_usd and os.path.exists(args.robot_usd):
            add_reference_to_stage(usd_path=args.robot_usd, prim_path=args.robot_prim)
            prim_utils.set_prim_attribute_value(
                args.robot_prim, "xformOp:translate", [0.0, 0.0, 0.65])
        else:
            carb.log_error("로봇 USD 미지정 (--robot-usd 또는 --usd 필요)")
            simulation_app.close()
            return
        simulation_app.update()

    # ── articulation 준비 (Nav 구동용) ────────────────────────────
    robot = SingleArticulation(prim_path=args.robot_prim, name="spot")
    world.reset()
    robot.initialize()
    simulation_app.update()

    # ── 센서 prim 생성(add_sensors) + OmniGraph 브릿지 ────────────
    prims = sensor_prims(args.robot_prim)
    if not args.no_sensors:
        # add_sensors / setup_semantics 모듈은 같은 폴더
        import sys
        sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
        try:
            import add_sensors            # 로봇에 RGB/Lidar360/IMU prim 생성
            add_sensors.add_sensors(args.robot_prim)
            simulation_app.update()
            setup_sensor_bridge(prims, auto_semantics=not args.no_semantics)
        except Exception as e:
            carb.log_error(f"센서 구성 실패: {e}")
            print("  ⚠️  센서 없이 Nav만 계속합니다.")
    else:
        print("[센서] --no-sensors → 센서 브릿지 건너뜀")

    # ── Nav 브릿지 ROS 2 노드 ─────────────────────────────────────
    rclpy.init()
    nav_node = IsaacNavBridge(robot, args.mode)

    print("\n[실행] 통합 브릿지 동작. 다른 터미널에서:")
    print("       ros2 launch smart_farm_spot bringup.launch.py")
    print("       또는 ros2 launch smart_farm_spot spot_nav2.launch.py\n")

    try:
        while simulation_app.is_running():
            world.step(render=True)        # render=True → 카메라/LiDAR 발행
            nav_node.update()              # cmd_vel 적용 + odom/TF 발행
            rclpy.spin_once(nav_node, timeout_sec=0.0)
    except KeyboardInterrupt:
        print("\n[종료] 사용자 중단")
    finally:
        nav_node.destroy_node()
        rclpy.shutdown()
        simulation_app.close()


if __name__ == "__main__":
    main()
