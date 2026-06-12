#!/usr/bin/env bash
# ─────────────────────────────────────────────────────────────────────────
# run_scenario_slam.sh — **진짜 SLAM** 버전: slam_toolbox 실시간 맵 작성
#   run_scenario.sh 와 동일하되 3단계만 교체:
#     [기존] barn_map_server (사전맵, map→odom identity)
#     [SLAM] slam_toolbox    (/scan → /map 실시간 작성 + map→odom 추정)
#
#   1) scenario.py (Isaac): RL 보행 + /cmd_vel 구독 + /scan + 카메라 + 소
#   2) slam_toolbox: /scan + odom→base_link → /map + map→odom (실시간 SLAM)
#   3) Nav2(nav2_params_slam): /map·/scan → 경로계획 → /cmd_vel
#   4) keepout 필터(투명벽 — SLAM 은 라이다 미감지 벽을 못 그리므로 보강)
#   5) yolo_view + scenario_nav(소 후방 주행)
#
# 사용: ./scripts/run_scenario_slam.sh
# 참고: SLAM 은 라이다가 본 벽만 맵에 그림(투명벽 X) → keepout 으로 보강.
#       매핑 후 저장:  ros2 run nav2_map_server map_saver_cli -f ~/barn_slam_map
# ─────────────────────────────────────────────────────────────────────────
set +u
DOMAIN=153
WS=/home/rokey/dev_ws/spot_ws
PKG="$WS/src/smart_farm_spot"
INST="$WS/install/smart_farm_spot/share/smart_farm_spot"
ISAACLAB=/home/rokey/dev_ws/isaac_sim/IsaacLab
VENV=/home/rokey/dev_ws/venv/isaaclab
FLATPOL="$PKG/assets/policy/policy_spot_flat.pt"   # 워크스페이스 자체완결
DISP="${DISPLAY:-:1}"

echo "════════════ 시나리오(진짜 SLAM): slam_toolbox + Nav2 + RL (도메인 $DOMAIN) ════════════"

# 0) 좀비 정리
echo "[0/5] 좀비 정리..."
ps -eo pid,comm | awk '/controll|planner|navigat|behavior|lifecycle|waypoint|velocit|slam_too|costmap|rviz|map_serv/ {print $1}' | xargs -r kill -9 2>/dev/null
MY=$$; ps -eo pid,args | grep "[s]cenario_nav.py\|[h]_drive.py\|[b]arn_map_server.py" | awk '{print $1}' | while read p; do [ "$p" != "$MY" ] && kill -9 "$p" 2>/dev/null; done
for p in $(nvidia-smi --query-compute-apps=pid --format=csv,noheader 2>/dev/null); do kill -9 "$p" 2>/dev/null; done
sleep 3

source /opt/ros/humble/setup.bash
[ -f "$WS/install/setup.bash" ] && source "$WS/install/setup.bash"
export ROS_DOMAIN_ID=$DOMAIN

# 1) Isaac 시나리오 브릿지
echo "[1/5] Isaac 브릿지 기동 (RL+/cmd_vel+/scan+카메라+소, 로딩 ~70s)..."
rm -f /tmp/scenario.log /tmp/scenario_goal.json
ROS_DOMAIN_ID=$DOMAIN PYTHONUNBUFFERED=1 TERM=xterm DISPLAY="$DISP" \
  SF_POLICY="$FLATPOL" SF_PEN_FRICTION=0.45 SF_CORRIDOR_FRICTION=0.7 SF_BEHIND_TAIL=1.5 \
  setsid bash -c "cd $ISAACLAB && source $VENV/bin/activate && exec ./isaaclab.sh -p $PKG/isaac/scenario.py" \
  > /tmp/scenario.log 2>&1 < /dev/null &
echo "    bridge pid $!"

# 2) 브릿지 ready 대기 (/scan)
echo "[2/5] 브릿지 ready 대기..."
until ROS_DOMAIN_ID=$DOMAIN ros2 topic list 2>/dev/null | grep -q "^/scan$"; do
  if grep -qE "Traceback|Aborted|CUDA unknown" /tmp/scenario.log 2>/dev/null; then
    echo "  ❌ 브릿지 기동 실패:"; tail -25 /tmp/scenario.log; exit 1
  fi
  sleep 5
done
sleep 6
echo "    ✅ /scan /odom /tf /spot_cam 발행"

# 3) ★ slam_toolbox — SF_SLAM_MODE 로 모드 선택 ★
#   mapping        : spot_slam.launch.py        (사전맵 없이 실시간 작성)
#   localization   : spot_slam_loc.launch.py    (사전맵 로드+위치추정+갱신 = AMCL+SLAM 복합) ← 기본
SF_SLAM_MODE="${SF_SLAM_MODE:-localization}"
rm -f /tmp/slam.log
if [ "$SF_SLAM_MODE" = "mapping" ]; then
  echo "[3/5] slam_toolbox 매핑 모드 (실시간 맵 작성)..."
  setsid ros2 launch smart_farm_spot spot_slam.launch.py use_sim_time:=true \
    > /tmp/slam.log 2>&1 < /dev/null &
else
  echo "[3/5] slam_toolbox 복합 모드 (AMCL 위치추정 + SLAM 갱신, 사전맵 로드)..."
  if [ ! -f "$INST/maps/barn_slam.posegraph" ]; then
    echo "    ⚠️ 직렬화 사전맵($INST/maps/barn_slam.posegraph) 없음 — 빈 맵에서 시작(매핑+위치추정)."
    echo "       사전맵 만들려면: SF_SLAM_MODE=mapping 으로 한 번 주행 후 serialize."
  fi
  setsid ros2 launch smart_farm_spot spot_slam_loc.launch.py use_sim_time:=true \
    map:="$INST/maps/barn_slam" > /tmp/slam.log 2>&1 < /dev/null &
fi
echo "    초기화 대기 12s..."; sleep 12
# /map + map→odom 확인
if ros2 topic list 2>/dev/null | grep -q "^/map$"; then
  echo "    ✅ /map 발행 (slam_toolbox 매핑 시작)"
else
  echo "    ⚠️ /map 미발행 — slam.log 확인:"; tail -10 /tmp/slam.log
fi

# 4) Nav2
echo "[4/5] Nav2 (라이다 코스트맵 경로계획)..."
rm -f /tmp/nav2.log
setsid ros2 launch smart_farm_spot nav2_flat.launch.py use_sim_time:=true autostart:=false \
  params_file:="$INST/config/nav2_params_slam.yaml" > /tmp/nav2.log 2>&1 < /dev/null &
echo "    노드 기동 대기 20s..."; sleep 20
ros2 service call /lifecycle_manager_navigation/manage_nodes nav2_msgs/srv/ManageLifecycleNodes "{command: 0}" 2>&1 | tail -1
for n in controller_server planner_server behavior_server bt_navigator waypoint_follower velocity_smoother; do
  st=$(timeout 6 ros2 lifecycle get /$n 2>/dev/null)
  echo "$st" | grep -q unconfigured && timeout 10 ros2 lifecycle set /$n configure >/dev/null 2>&1
  st=$(timeout 6 ros2 lifecycle get /$n 2>/dev/null)
  echo "$st" | grep -q inactive && timeout 10 ros2 lifecycle set /$n activate >/dev/null 2>&1
  echo "    $n: $(timeout 6 ros2 lifecycle get /$n 2>/dev/null)"
done
sleep 3

# 4b) Keepout 필터 — SLAM 은 투명벽을 못 그리므로 보강(+nav2_params_slam 필수). 기본 ON.
if [ "${SF_KEEPOUT:-1}" = "1" ]; then
  echo "[4b] Keepout 필터 서버 (투명벽 보강 + 코스트맵 필수)..."
  if [ ! -f "$INST/maps/keepout_mask.yaml" ]; then
    echo "    keepout 마스크 생성..."
    ROS_DOMAIN_ID=$DOMAIN python3 "$PKG/tools/make_keepout_mask.py" 2>/dev/null
    cp -f "$PKG/maps/keepout_mask.pgm" "$PKG/maps/keepout_mask.yaml" "$INST/maps/" 2>/dev/null
  fi
  rm -f /tmp/keepout.log
  setsid ros2 launch smart_farm_spot keepout.launch.py > /tmp/keepout.log 2>&1 < /dev/null &
  sleep 4
fi

# 5) YOLO + 소 후방 목표전송
echo "[5/5] YOLO + 소 후방 목표전송..."
setsid python3 "$PKG/yolo_view.py" > /tmp/yolo.log 2>&1 < /dev/null &
echo "    YOLO pid $! → /tmp/yolo.log (rqt_image_view /yolo/annotated)"
if [ "${SF_VISION_TAIL:-0}" = "1" ]; then
  echo "    ▶ in-Isaac 비전 목표 → scenario_nav.py relay"
  ROS_DOMAIN_ID=$DOMAIN python3 "$PKG/scenario_nav.py"
elif [ "${SF_PATROL:-0}" = "1" ]; then
  echo "    ▶ patrol.py: 통로 끝 전부 순찰(SLAM+Nav2+RL)"
  ROS_DOMAIN_ID=$DOMAIN python3 "$PKG/patrol.py"
else
  echo "    ▶ scenario_nav.py: 소 후방(꼬리 1.5m 뒤)로 Nav2 주행"
  ROS_DOMAIN_ID=$DOMAIN python3 "$PKG/scenario_nav.py"
fi
echo "════════════ SLAM 시나리오 종료 ════════════"
