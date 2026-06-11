#!/usr/bin/env bash
# ─────────────────────────────────────────────────────────────────────────
# run_slam_nav.sh — RL 보행정책(235차원) + 축사 + 라이다 + SLAM + Nav2 H코스
#   1) nav_policy_slam_bridge.py (Isaac): RL 보행 + 험지(235차원) + 마찰존
#      + RTX 라이다(/scan) + odom/tf(odom→base). map→odom 은 SLAM 이 발행.
#   2) slam_toolbox: /scan → /map + map→odom
#   3) Nav2(온라인 SLAM params): /map 정적층 + /scan 장애물층
#   4) h_drive: 끝→대각선 끝 (waypoints_diag.yaml)
#
# 사용: ./scripts/run_slam_nav.sh
# ─────────────────────────────────────────────────────────────────────────

DOMAIN="${ROS_DOMAIN_ID:-153}"
[ "$DOMAIN" = "0" ] && DOMAIN=153
WS=/home/rokey/dev_ws/spot_ws
PKG="$WS/src/smart_farm_spot"
INST="$WS/install/smart_farm_spot/share/smart_farm_spot"
ISAACLAB=/home/rokey/dev_ws/isaac_sim/IsaacLab
VENV=/home/rokey/dev_ws/venv/isaaclab
DISP="${DISPLAY:-:1}"

echo "════════════════════════════════════════════════════════════"
echo " [RL+SLAM 주행]  ROS_DOMAIN_ID=$DOMAIN"
echo "   235차원 험지 + 마찰존(통로 ${SF_CORRIDOR_FRICTION:-0.7}/우리 ${SF_PEN_FRICTION:-0.45})"
echo "   끝→대각선 끝 (waypoints_diag.yaml)"
echo "════════════════════════════════════════════════════════════"

echo "[1/5] 기존 프로세스 정리..."
pkill -9 -f "nav_policy_slam_bridge.py" 2>/dev/null || true
pkill -9 -f "nav_bridge.py" 2>/dev/null || true
pkill -9 -f "slam_toolbox" 2>/dev/null || true
pkill -9 -f "yolo_view.py" 2>/dev/null || true
sleep 3

source /opt/ros/humble/setup.bash
[ -f "$WS/install/setup.bash" ] && source "$WS/install/setup.bash"
export ROS_DOMAIN_ID=$DOMAIN

echo "[2/5] Isaac RL+SLAM 브릿지 기동 (로딩 ~60s)..."
rm -f /tmp/navslam.log
PYTHONUNBUFFERED=1 TERM=xterm DISPLAY="$DISP" ROS_DOMAIN_ID=$DOMAIN \
  SF_CORRIDOR_FRICTION="${SF_CORRIDOR_FRICTION:-0.7}" \
  SF_PEN_FRICTION="${SF_PEN_FRICTION:-0.45}" \
  SF_PEN_REGION="${SF_PEN_REGION:-1.0,1.5,0.9,0.9}" \
  SF_BARN="${SF_BARN:-1}" \
  setsid bash -c "cd $ISAACLAB && source $VENV/bin/activate && exec ./isaaclab.sh -p $PKG/isaac/nav_policy_slam_bridge.py" \
  > /tmp/navslam.log 2>&1 < /dev/null &
echo "        브릿지 pid $! → /tmp/navslam.log"

echo "[3/5] 브릿지 ready 대기 (OmniGraph)..."
until grep -qE "OmniGraph ROS2:|Traceback|Killed|Error: |CUDA" /tmp/navslam.log 2>/dev/null; do sleep 5; done
if grep -qE "Traceback|Killed|CUDA unknown" /tmp/navslam.log; then
  echo "  ❌ 브릿지 기동 실패 — /tmp/navslam.log 확인"; tail -20 /tmp/navslam.log; exit 1
fi
echo "  ✅ 브릿지 ready. /scan /odom 발행 시작."
sleep 5

echo "[4/5] SLAM + Nav2 기동..."
rm -f /tmp/slam.log /tmp/nav2slam.log
setsid ros2 launch smart_farm_spot spot_slam.launch.py use_sim_time:=true \
  > /tmp/slam.log 2>&1 < /dev/null &
echo "        slam_toolbox pid $! → /tmp/slam.log"
sleep 3
setsid ros2 launch smart_farm_spot nav2_flat.launch.py use_sim_time:=true \
  params_file:="$INST/config/nav2_params_slam.yaml" \
  > /tmp/nav2slam.log 2>&1 < /dev/null &
echo "        nav2 pid $! → /tmp/nav2slam.log"

echo "[5/5] Nav2 활성 대기..."
until grep -qE "Creating bond timer|Activating|bond" /tmp/nav2slam.log 2>/dev/null; do sleep 3; done
sleep 8
echo "  ✅ Nav2 준비. 끝→대각선 끝 주행 시작."
echo ""
echo "▶ H 대각선 주행 전송:"
echo "   ros2 topic echo /scan --once   (라이다 확인)"
echo "   rqt 또는 rviz2 로 /map 확인"
ROS_DOMAIN_ID=$DOMAIN python3 "$PKG/h_drive.py" \
  --ros-args -p waypoints:="$PKG/config/waypoints_diag.yaml"
