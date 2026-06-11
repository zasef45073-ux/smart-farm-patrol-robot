#!/usr/bin/env python3
# Copyright (c) 2024, Smart Farm Patrol Project - 꼬마 로봇 두리
# SPDX-License-Identifier: Apache-2.0
"""
ros_bridge_test.py
==================
[마일스톤 0] Isaac OmniGraph ROS2 브릿지 ↔ 시스템 ROS2(Nav2) DDS 연결 검증.

rclpy 없이 OmniGraph 의 ROS2PublishClock 로 /clock 만 발행.
다른 터미널(시스템 humble)에서:
    source /opt/ros/humble/setup.bash
    ros2 topic list            # /clock 보이면 성공
    ros2 topic echo /clock     # 시간 증가하면 DDS 연결 OK

실행:
    source ~/dev_ws/venv/isaaclab/bin/activate
    cd ~/dev_ws/isaac_sim/IsaacLab
    ./isaaclab.sh -p ~/dev_ws/spot_ws/src/smart_farm_spot/isaac/ros_bridge_test.py
"""

import argparse
import os

# ROS2 도메인 강제 (isaaclab.sh 가 0 으로 덮어쓰므로 여기서 고정).
# ros2 bridge 확장 로드 전에 설정해야 적용됨.
os.environ["ROS_DOMAIN_ID"] = "153"

parser = argparse.ArgumentParser()
parser.add_argument("--headless", action="store_true")
a = parser.parse_args()

from isaaclab.app import AppLauncher  # noqa: E402
app = AppLauncher(headless=a.headless).app

from isaacsim.core.utils.extensions import enable_extension  # noqa: E402
enable_extension("omni.graph.action")
enable_extension("isaacsim.ros2.bridge")
app.update()

import omni.graph.core as og  # noqa: E402
from isaacsim.core.api import World  # noqa: E402

NS_ROS2 = "isaacsim.ros2.bridge"
NS_CORE = "isaacsim.core.nodes"


def main():
    print("=" * 60)
    print(" [마일스톤0] Isaac→시스템 ROS2 DDS 연결 테스트 (/clock)")
    print(f"   ROS_DOMAIN_ID(런타임) = {os.getenv('ROS_DOMAIN_ID')}")
    print("=" * 60)

    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()
    world.reset()                 # 그래프 생성 전에 스테이지/물리 준비
    app.update()

    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": "/ROS2ClockTest", "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("Tick",     "omni.graph.action.OnPlaybackTick"),
                ("SimTime",  f"{NS_CORE}.IsaacReadSimulationTime"),
                ("ClockPub", f"{NS_ROS2}.ROS2PublishClock"),
            ],
            keys.SET_VALUES: [
                ("ClockPub.inputs:topicName", "/clock"),
            ],
            keys.CONNECT: [
                ("Tick.outputs:tick", "ClockPub.inputs:execIn"),
                ("SimTime.outputs:simulationTime", "ClockPub.inputs:timeStamp"),
            ],
        },
    )
    print("  ✅ OmniGraph ROS2PublishClock 생성 → /clock")

    print("\n[실행] /clock 발행 중. 다른 터미널(시스템 ROS2)에서:")
    print("   source /opt/ros/humble/setup.bash")
    print("   ros2 topic list    (/clock 보이면 DDS 연결 성공)")
    print("   ros2 topic echo /clock --once\n")

    try:
        step = 0
        while app.is_running():
            world.step(render=True)
            step += 1
            if step % 200 == 0:
                print(f"  ... {step} 스텝 발행 중")
    except KeyboardInterrupt:
        print("\n[종료]")
    finally:
        app.close()


if __name__ == "__main__":
    main()
