#!/usr/bin/env bash
# ─────────────────────────────────────────────────────────────────────────
# run_scenario_amcl.sh — **AMCL 위치추정** 버전 (사전맵 + 파티클필터)
#   run_scenario.sh 와 동일하되:
#     [기존] barn_map_server: /map + map→odom identity
#     [AMCL] barn_map_server: /map 만(SF_NO_MAP_TF=1) + AMCL 이 map→odom 추정
#
#   1) scenario.py (Isaac): RL 보행 + /cmd_vel + /scan + 카메라 + 소
#   2) barn_map_server(SF_NO_MAP_TF=1): /map 만 (TF 는 AMCL 에 양보)
#   3) Nav2+AMCL(nav2_amcl.launch.py, nav2_params_amcl): /scan+사전맵 → map→odom 추정 → 경로계획
#   4) keepout 필터(투명벽)
#   5) yolo_view + scenario_nav(소 후방)
#
# 사용: ./scripts/run_scenario_amcl.sh
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

echo "════════════ 시나리오(AMCL): 사전맵 + AMCL 위치추정 + Nav2 + RL (도메인 $DOMAIN) ════════════"

# 0) 좀비 정리
echo "[0/5] 좀비 정리..."
ps -eo pid,comm | awk '/controll|planner|navigat|behavior|lifecycle|waypoint|velocit|slam_too|costmap|rviz|map_serv|amcl/ {print $1}' | xargs -r kill -9 2>/dev/null
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

# 3) barn_map_server — /map 만 (SF_NO_MAP_TF=1: map→odom 은 AMCL 에 양보)
echo "[3/5] barn_map_server (/map 만, TF 는 AMCL)..."
rm -f /tmp/mapserver.log
ROS_DOMAIN_ID=$DOMAIN SF_NO_MAP_TF=1 setsid python3 "$PKG/barn_map_server.py" > /tmp/mapserver.log 2>&1 < /dev/null &
sleep 5

# 4) Nav2 + AMCL
echo "[4/5] Nav2 + AMCL (사전맵 위치추정 + 경로계획)..."
rm -f /tmp/nav2.log
setsid ros2 launch smart_farm_spot nav2_amcl.launch.py use_sim_time:=true autostart:=false \
  params_file:="$INST/config/nav2_params_amcl.yaml" > /tmp/nav2.log 2>&1 < /dev/null &
echo "    노드 기동 대기 20s..."; sleep 20
ros2 service call /lifecycle_manager_navigation/manage_nodes nav2_msgs/srv/ManageLifecycleNodes "{command: 0}" 2>&1 | tail -1
for n in amcl controller_server planner_server behavior_server bt_navigator waypoint_follower velocity_smoother; do
  st=$(timeout 6 ros2 lifecycle get /$n 2>/dev/null)
  echo "$st" | grep -q unconfigured && timeout 10 ros2 lifecycle set /$n configure >/dev/null 2>&1
  st=$(timeout 6 ros2 lifecycle get /$n 2>/dev/null)
  echo "$st" | grep -q inactive && timeout 10 ros2 lifecycle set /$n activate >/dev/null 2>&1
  echo "    $n: $(timeout 6 ros2 lifecycle get /$n 2>/dev/null)"
done
sleep 3
# AMCL 수렴 확인(map→odom 발행 여부)
ros2 run tf2_ros tf2_echo map odom 2>/dev/null | grep -m1 -A1 Translation | head -2 || echo "    ⚠️ AMCL map→odom 미발행(수렴 대기)"

# 4b) Keepout 필터 (투명벽 + 코스트맵 필수). 기본 ON.
if [ "${SF_KEEPOUT:-1}" = "1" ]; then
  echo "[4b] Keepout 필터 서버..."
  if [ ! -f "$INST/maps/keepout_mask.yaml" ]; then
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
echo "    YOLO pid $! → /tmp/yolo.log"
if [ "${SF_VISION_TAIL:-0}" = "1" ]; then
  ROS_DOMAIN_ID=$DOMAIN python3 "$PKG/scenario_nav.py"
elif [ "${SF_PATROL:-0}" = "1" ]; then
  ROS_DOMAIN_ID=$DOMAIN python3 "$PKG/patrol.py"
else
  ROS_DOMAIN_ID=$DOMAIN python3 "$PKG/scenario_nav.py"
fi
echo "════════════ AMCL 시나리오 종료 ════════════"
