#!/usr/bin/env bash
# ─────────────────────────────────────────────────────────────────────────
# run_scenario.sh — 시나리오: SLAM + Nav2 + RL 주행 → 소 후방(꼬리) 이동
#   1) scenario.py (Isaac): RL 정책 보행 + /cmd_vel(Nav2) 구독 + /scan(라이다)
#      + 카메라(YOLO) + 축사(구역마찰·가상벽) + 텍스처 소(통로 교차점)
#   2) slam_toolbox: /scan → /map + map→odom
#   3) Nav2(nav2_params_slam): /map·/scan 코스트맵 경로계획·장애물회피 → /cmd_vel
#   4) scenario_nav.py: 소 후방(꼬리 1.5m 뒤) NavigateToPose 목표전송
#   5) yolo_view.py: 소 검출 표시
#  모든 노드 ROS_DOMAIN_ID=153 (명시).
#
# 사용: ./scripts/run_scenario.sh
# ─────────────────────────────────────────────────────────────────────────
set +u
DOMAIN=153
WS=/home/rokey/dev_ws/spot_ws
PKG="$WS/src/smart_farm_spot"
INST="$WS/install/smart_farm_spot/share/smart_farm_spot"
ISAACLAB=/home/rokey/dev_ws/isaac_sim/IsaacLab
VENV=/home/rokey/dev_ws/venv/isaaclab
# 무팔 정책(no_arm) — 워크스페이스 자체완결(assets/policy/). 외부 IsaacLab 로그 의존 제거.
FLATPOL="$PKG/assets/policy/policy_no_arm_bast.pt"
DISP="${DISPLAY:-:1}"

echo "════════════ 시나리오: SLAM + Nav2 + RL_주행 (도메인 $DOMAIN) ════════════"

# 0) 좀비 정리 (comm 매칭 = 안전)
echo "[0/5] 좀비 정리..."
ps -eo pid,comm | awk '/controll|planner|navigat|behavior|lifecycle|waypoint|velocit|slam_too|costmap|rviz|map_serv/ {print $1}' | xargs -r kill -9 2>/dev/null
MY=$$; ps -eo pid,args | grep "[s]cenario_nav.py\|[h]_drive.py" | awk '{print $1}' | while read p; do [ "$p" != "$MY" ] && kill -9 "$p" 2>/dev/null; done
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

# 3) 알려진 맵 서버 (SLAM 대체 — Isaac 추출 Prior Map: 펜스+투명벽, map→odom identity)
echo "[3/5] barn_map_server (알려진 맵 /map + map→odom)..."
rm -f /tmp/mapserver.log
ROS_DOMAIN_ID=$DOMAIN setsid python3 "$PKG/barn_map_server.py" > /tmp/mapserver.log 2>&1 < /dev/null &
sleep 5

# 4) Nav2 (autostart=false → 수동 startup: 활성 레이스 회피)
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

# 4b) Keepout 필터 서버 — nav2_params_slam 의 keepout_filter 가 /costmap_filter_info 를
#  필수로 요구. 안 띄우면 코스트맵이 마스크를 영원히 기다려 막힘 → controller "Failed to
#  make progress" → 주행 정체. 따라서 **기본 ON**. (SF_KEEPOUT=0 으로만 끔)
if [ "${SF_KEEPOUT:-1}" = "1" ]; then
  echo "[4b] Keepout 필터 서버 (투명벽 회피 + 코스트맵 필수)..."
  # 마스크 없으면 생성 후 install share 로 복사(keepout.launch.py 가 share 에서 읽음)
  if [ ! -f "$INST/maps/keepout_mask.yaml" ]; then
    echo "    keepout 마스크 생성..."
    ROS_DOMAIN_ID=$DOMAIN python3 "$PKG/tools/make_keepout_mask.py" 2>/dev/null
    cp -f "$PKG/maps/keepout_mask.pgm" "$PKG/maps/keepout_mask.yaml" "$INST/maps/" 2>/dev/null
  fi
  rm -f /tmp/keepout.log
  setsid ros2 launch smart_farm_spot keepout.launch.py > /tmp/keepout.log 2>&1 < /dev/null &
  sleep 4
fi

# 4c) (선택) 웹 대시보드 브리지 — /patrol/command ↔ /robot/status (SF_DASHBOARD=1)
if [ "${SF_DASHBOARD:-0}" = "1" ]; then
  echo "[4c] 대시보드 브리지 (웹 ↔ 로봇)..."
  rm -f /tmp/dashboard_bridge.log
  ROS_DOMAIN_ID=$DOMAIN setsid python3 "$PKG/dashboard_bridge.py" > /tmp/dashboard_bridge.log 2>&1 < /dev/null &
  echo "    bridge pid $! → /tmp/dashboard_bridge.log"
fi

# 5) YOLO + 소 후방 목표전송(NavigateToPose)
echo "[5/5] YOLO + 소 후방 목표전송..."
setsid python3 "$PKG/yolo_view.py" > /tmp/yolo.log 2>&1 < /dev/null &
echo "    YOLO pid $! → /tmp/yolo.log (rqt_image_view /yolo/annotated)"
if [ "${SF_VISION_TAIL:-0}" = "1" ]; then
  echo "    ▶ in-Isaac 비전: scenario.py 내부 YOLO 검출이 목표 생성 → scenario_nav.py relay"
  ROS_DOMAIN_ID=$DOMAIN python3 "$PKG/scenario_nav.py"
elif [ "${SF_PATROL:-0}" = "1" ]; then
  echo "    ▶ patrol.py: 통로 끝 전부 순찰(SLAM+Nav2+RL)"
  ROS_DOMAIN_ID=$DOMAIN python3 "$PKG/patrol.py"
else
  echo "    ▶ scenario_nav.py: 소 후방(꼬리 1.5m 뒤)로 Nav2 주행"
  ROS_DOMAIN_ID=$DOMAIN python3 "$PKG/scenario_nav.py"
fi
echo "════════════ 시나리오 종료 ════════════"
