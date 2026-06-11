#!/usr/bin/env bash
# ─────────────────────────────────────────────────────────────────────────
# run_local.sh — 로컬 단독 모드 (서버 불필요, 완전 자립)
#   에셋: 워크스페이스 assets/scene (Nucleus 불필요)
#   Nav2: 워크스페이스 nav2_flat.launch.py 를 직접 실행 (깨끗한 Nav2)
#   = 팀 서버가 없어도 노트북 한 대로 시나리오 전체 실행
#
# 사용: ./scripts/run_local.sh            (또는 ROS_DOMAIN_ID=153 ./run_local.sh)
# ─────────────────────────────────────────────────────────────────────────
# (set -u 금지 — ROS setup.bash 가 unbound 변수 사용)
# 로컬 단독은 서버(153)와 격리되도록 기본 도메인 154. 153 쓰려면 서버 Nav2 를 끄고
#   ROS_DOMAIN_ID=153 ./run_local.sh 로 실행. (.bashrc 가 0 강제하므로 0 도 154 로)
DOMAIN="${ROS_DOMAIN_ID:-154}"
[ "$DOMAIN" = "0" ] && DOMAIN=154
WS=/home/rokey/dev_ws/spot_ws
PKG="$WS/src/smart_farm_spot"
ISAACLAB=/home/rokey/dev_ws/isaac_sim/IsaacLab
VENV=/home/rokey/dev_ws/venv/isaaclab
DISP="${DISPLAY:-:1}"

echo "════════════════════════════════════════════════════════════"
echo " [로컬 단독 모드]  ROS_DOMAIN_ID=$DOMAIN"
echo "   에셋 = 로컬(assets/scene)   |   Nav2 = 로컬 직접 실행"
echo "════════════════════════════════════════════════════════════"

# 1) 기존 프로세스 정리
echo "[1/4] 기존 브릿지/YOLO/Nav2 정리..."
for pat in "nav_bridge.py" "yolo_view.py" "nav2_flat.launch" "controller_server" "planner_server"; do
  pkill -9 -f "$pat" 2>/dev/null || true
done
sleep 3

source /opt/ros/humble/setup.bash
[ -f "$WS/install/setup.bash" ] && source "$WS/install/setup.bash"
export ROS_DOMAIN_ID=$DOMAIN

# 1-b) 이 도메인에 이미 Nav2 가 떠 있으면 경고(중복 충돌 방지)
if ros2 node list 2>/dev/null | grep -q controller_server; then
  echo "  ⚠️ 도메인 $DOMAIN 에 Nav2(controller_server) 가 이미 있습니다!"
  echo "     중복되면 충돌합니다 → 그 Nav2 를 끄거나 다른 도메인을 쓰세요."
  echo "     (서버 Nav2 와 격리하려면 기본 도메인 154 사용)"
fi

# 2) 로컬 Nav2 스택 (map/amcl 없이 controller/planner/bt/behavior/smoother/waypoint)
echo "[2/4] 로컬 Nav2 실행 (nav2_flat.launch.py)..."
setsid ros2 launch smart_farm_spot nav2_flat.launch.py \
  > /tmp/nav2_local.log 2>&1 < /dev/null &
echo "        Nav2 pid $! → /tmp/nav2_local.log"

# 3) Isaac 브릿지 (로컬 에셋 + RealSense + 라이다 + 텍스처 소)
echo "[3/4] Isaac 브릿지 실행 (로컬 에셋)... Isaac 로딩 ~60s"
rm -f /tmp/navlight.log
PYTHONUNBUFFERED=1 TERM=xterm DISPLAY="$DISP" ROS_DOMAIN_ID=$DOMAIN SF_DOMAIN=$DOMAIN \
  SF_ASSET_BASE=local SF_STAND_Z="${SF_STAND_Z:-0.90}" \
  SF_RS_WORLD="${SF_RS_WORLD:-0.0,0.0,0.06}" SF_COW_YAW="${SF_COW_YAW:-0}" \
  setsid bash -c "cd $ISAACLAB && source $VENV/bin/activate && exec ./isaaclab.sh -p $PKG/isaac/nav_bridge.py" \
  > /tmp/navlight.log 2>&1 < /dev/null &
echo "        브릿지 pid $! → /tmp/navlight.log"

# 4) 브릿지 ready 대기 후 YOLO
echo "[4/4] 브릿지 ready 대기..."
until grep -qE "OmniGraph: |Traceback|Killed|Error: " /tmp/navlight.log 2>/dev/null; do sleep 5; done
if grep -qE "Traceback|Killed|Error: " /tmp/navlight.log; then
  echo "  ❌ 브릿지 기동 실패 — /tmp/navlight.log 확인"; exit 1
fi
echo "  ✅ 브릿지 ready. YOLO 검출 시작."
setsid python3 "$PKG/yolo_view.py" > /tmp/yolo.log 2>&1 < /dev/null &
echo "        YOLO pid $! → /tmp/yolo.log  (rqt_image_view /yolo/annotated 로 확인)"

echo ""
echo "✅ 로컬 단독 모드 가동 완료."
echo "   주행:  ROS_DOMAIN_ID=$DOMAIN python3 $PKG/h_drive.py          (H 코스)"
echo "          ROS_DOMAIN_ID=$DOMAIN python3 $PKG/nav_to_cow.py       (소 접근)"
echo "   결과:  rqt_image_view /yolo/annotated"
