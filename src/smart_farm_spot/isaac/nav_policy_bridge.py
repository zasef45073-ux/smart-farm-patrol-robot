#!/usr/bin/env python3
# Copyright (c) 2024, Smart Farm Patrol Project - 꼬마 로봇 두리
# SPDX-License-Identifier: Apache-2.0
"""
nav_policy_bridge.py
====================
강화학습 보행 정책 + Nav2 자율주행 브릿지.

nav_bridge.py(kinematic 미끄러짐) 대신, IsaacLab RL env 로 로봇을 띄워
학습된 보행 정책으로 **실제 다리 보행**을 하고, Nav2 의 /cmd_vel 을
env 의 속도명령(base_velocity)으로 주입한다. env 가 235차원 관측을 정확히
계산하므로 정책이 제대로 동작.

ROS 인터페이스(rclpy 없이 OmniGraph):
  구독 /cmd_vel  → base_velocity 명령 주입
  발행 /odom, /tf(odom→base_link, map→odom), /clock

실행:
  source ~/dev_ws/venv/isaaclab/bin/activate
  cd ~/dev_ws/isaac_sim/IsaacLab
  ./isaaclab.sh -p ~/dev_ws/spot_ws/src/smart_farm_spot/isaac/nav_policy_bridge.py
"""

import argparse
import math
import os

os.environ["ROS_DOMAIN_ID"] = os.environ.get("ROS_DOMAIN_ID", "153")

_ASSETS = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "assets")
TASK = "Isaac-Velocity-Rough-SpotArm-Play-v0"
POLICY = os.path.join(_ASSETS, "policy", "policy.pt")

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
import omni.graph.core as og  # noqa: E402
from isaacsim.core.utils.extensions import enable_extension  # noqa: E402

import isaaclab_tasks  # noqa: F401,E402  (태스크 등록)
from isaaclab_tasks.utils import parse_env_cfg  # noqa: E402

NS_ROS2 = "isaacsim.ros2.bridge"
NS_CORE = "isaacsim.core.nodes"
GRAPH = "/NavPolicyBridge"
SPAWN_Z_FRAME = "base_link"


def _yaw_from_quat(qw, qx, qy, qz):
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def build_ros_graph():
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
                ("TfMap.inputs:topicName", "/tf_static"),
                ("TfMap.inputs:parentFrameId", "map"),
                ("TfMap.inputs:childFrameId", "odom"),
                ("TfMap.inputs:staticPublisher", True),
            ],
            keys.CONNECT: [
                ("Tick.outputs:tick", "ClockPub.inputs:execIn"),
                ("Tick.outputs:tick", "SubTwist.inputs:execIn"),
                ("Tick.outputs:tick", "OdomPub.inputs:execIn"),
                ("Tick.outputs:tick", "TfOdom.inputs:execIn"),
                ("Tick.outputs:tick", "TfMap.inputs:execIn"),
                ("SimTime.outputs:simulationTime", "ClockPub.inputs:timeStamp"),
                ("SimTime.outputs:simulationTime", "OdomPub.inputs:timeStamp"),
                ("SimTime.outputs:simulationTime", "TfOdom.inputs:timeStamp"),
                ("SimTime.outputs:simulationTime", "TfMap.inputs:timeStamp"),
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
    print(" RL 보행정책 + Nav2 브릿지")
    print(f"   task={a.task}  domain={os.getenv('ROS_DOMAIN_ID')}")
    print("=" * 60)

    # ── env 생성 (1 env, GUI) ─────────────────────────────────────
    env_cfg = parse_env_cfg(a.task, device="cuda:0", num_envs=1)
    # 기본: 학습 험지 terrain 유지 → height_scan(187) 의미있는 235차원.
    #  (축사 배변 우리 등 단차/험지 보행에 필요). SF_FLAT=1 이면 평면으로.
    if os.environ.get("SF_FLAT", "0") == "1":
        env_cfg.scene.terrain.terrain_type = "plane"
        env_cfg.scene.terrain.terrain_generator = None
        try:
            env_cfg.curriculum.terrain_levels = None
        except Exception:
            pass
        print("  ✅ 맵: 넓은 평면(plane)")
    else:
        print("  ✅ 맵: 학습 험지(rough terrain) — 235차원 height_scan 유효")
    env = gym.make(a.task, cfg=env_cfg)
    obs, _ = env.reset()

    # ── 정책 로드 (TorchScript) — env device(cuda)로 ──────────────
    device = env.unwrapped.device
    policy = torch.jit.load(a.policy, map_location=device)
    policy.eval()
    print(f"  ✅ 정책 로드: {a.policy} (device={device})")

    # ── ROS2 브릿지 ───────────────────────────────────────────────
    enable_extension("isaacsim.ros2.bridge")
    simulation_app.update()
    attrs = build_ros_graph()
    print("  ✅ OmniGraph ROS2: /clock /cmd_vel(sub) /odom /tf")

    robot = env.unwrapped.scene["robot"]
    cmd_term = env.unwrapped.command_manager.get_term("base_velocity")
    # 속도명령 자동 리샘플링 끔 → Nav2 /cmd_vel 만 따르게
    try:
        cmd_term.cfg.resampling_time_range = (1.0e9, 1.0e9)
    except Exception:
        pass
    # 팔은 수동 강제 안 함 — env 의 고강성 ImplicitActuator(stiffness=800)가
    # 학습 init 자세(sh1=-0.5, el0=0.0)를 자동으로 잡는다. 수동 강제는 학습 CoM 과
    # 어긋나 넘어짐의 원인이었으므로 제거.
    x0 = y0 = None   # odom 시작점 오프셋

    def policy_obs(o):
        # gym obs dict 에서 policy 그룹 → env device 로
        t = o["policy"] if isinstance(o, dict) else o
        return t.to(device)

    print("\n[실행] 정책 보행 + Nav2. 시스템 ROS2(도메인153)에서:")
    print("   ros2 launch smart_farm_spot nav2_flat.launch.py\n")

    step = 0
    while simulation_app.is_running():
        # ── /cmd_vel 읽어 base_velocity 명령 주입 ──
        lin = og.Controller.get(attrs["sub_lin"])
        ang = og.Controller.get(attrs["sub_ang"])
        vx = float(lin[0]) if lin is not None else 0.0
        wz = float(ang[2]) if ang is not None else 0.0
        cmd_term.vel_command_b[:, 0] = vx
        cmd_term.vel_command_b[:, 1] = 0.0
        cmd_term.vel_command_b[:, 2] = wz

        # ── 정책 추론 → env step (로봇 보행) ──
        with torch.inference_mode():
            actions = policy(policy_obs(obs))
            obs, _, _, _, _ = env.step(actions)

        # ── odom/TF 발행 (시작점 기준 상대 평면 포즈) ──
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
