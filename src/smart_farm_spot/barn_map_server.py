#!/usr/bin/env python3
# Copyright (c) 2024, Smart Farm Patrol Project - 꼬마 로봇 두리
# SPDX-License-Identifier: Apache-2.0
"""
barn_map_server.py
==================
**알려진 축사 맵** 서버 — slam_toolbox 대체.

scenario.py(브릿지)가 축사 콜라이더(펜스 + **투명 가상벽** 포함)를 래스터화해 저장한
  /tmp/barn_map.npy   : 점유격자 (H×W, int8: -1 미지 / 0 자유 / 100 점유)
  /tmp/barn_map.json  : {resolution, width, height, origin[x,y]} (origin=시작점 상대)
를 읽어 **nav_msgs/OccupancyGrid 를 /map 으로 latched 발행** + **map→odom static TF(identity)** 발행.

라이다 1m 사각·투명 벽과 무관하게 **실제 알려진 통로**를 Nav2 에 제공한다.
(시뮬 odom 이 정확하므로 map=odom 원점=로봇 시작점, 보정 불필요.)

실행:
  ROS_DOMAIN_ID=153 python3 barn_map_server.py
"""
import json
import os
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster

MAP_NPY = "/tmp/barn_map.npy"
MAP_JSON = "/tmp/barn_map.json"


class BarnMapServer(Node):
    def __init__(self):
        super().__init__("barn_map_server")
        for _ in range(60):
            if os.path.exists(MAP_NPY) and os.path.exists(MAP_JSON):
                break
            time.sleep(1.0)
        grid = np.load(MAP_NPY)
        with open(MAP_JSON) as f:
            meta = json.load(f)
        self.res = float(meta["resolution"])
        self.ox, self.oy = meta["origin"]
        self.H, self.W = int(grid.shape[0]), int(grid.shape[1])
        # latched(map 은 한 번만 받아도 유지) QoS
        qos = QoSProfile(depth=1)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = QoSReliabilityPolicy.RELIABLE
        self.pub = self.create_publisher(OccupancyGrid, "/map", qos)

        self.msg = OccupancyGrid()
        self.msg.header.frame_id = "map"
        self.msg.info.resolution = self.res
        self.msg.info.width = self.W
        self.msg.info.height = self.H
        self.msg.info.origin.position.x = float(self.ox)
        self.msg.info.origin.position.y = float(self.oy)
        self.msg.info.origin.orientation.w = 1.0
        self.msg.data = [int(v) for v in grid.flatten(order="C").tolist()]

        # map→odom = identity (시뮬 odom 이 정확). 단 SF_NO_MAP_TF=1 이면 생략 →
        #  AMCL 이 map→odom 을 추정(파티클필터)하도록 양보(AMCL 모드).
        self._no_tf = os.environ.get("SF_NO_MAP_TF", "0") == "1"
        if not self._no_tf:
            self.tf_static = StaticTransformBroadcaster(self)
            t = TransformStamped()
            t.header.frame_id = "map"
            t.child_frame_id = "odom"
            t.transform.rotation.w = 1.0
            self.tf_static.sendTransform(t)

        nocc = int((grid == 100).sum())
        self.get_logger().info(
            f"알려진 맵 발행: {self.W}×{self.H}@{self.res}m 점유셀{nocc}(투명벽 포함), "
            f"origin=({self.ox:.1f},{self.oy:.1f}) | "
            f"map→odom={'AMCL 추정(생략)' if self._no_tf else 'identity'}")
        # 주기 재발행(늦게 뜨는 구독자 대비)
        self.timer = self.create_timer(2.0, self._pub)
        self._pub()

    def _pub(self):
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.msg.info.map_load_time = self.msg.header.stamp
        self.pub.publish(self.msg)


def main():
    rclpy.init()
    try:
        rclpy.spin(BarnMapServer())
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
