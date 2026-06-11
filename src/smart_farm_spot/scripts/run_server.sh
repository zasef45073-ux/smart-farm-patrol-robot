#!/usr/bin/env bash
# ─────────────────────────────────────────────────────────────────────────
# run_server.sh — 팀 서버 모드 (Nucleus 에셋 + 서버 Nav2)
#   에셋: Nucleus omniverse://192.168.10.44/Projects/team1/environment
#   Nav2: 팀 서버에서 이미 실행 중인 Nav2 사용 (여기선 안 띄움)
#   = 팀원과 동일 Nucleus/서버 공유 환경
#
# 사용: ./scripts/run_server.sh
# 주의: 서버 Nav2 가 떠 있어야 함. 에셋은 Nucleus 접속 필요(192.168.10.44).
# ─────────────────────────────────────────────────────────────────────────

DOMAIN="${ROS_DOMAIN_ID:-153}"
[ "$DOMAIN" = "0" ] && DOMAIN=153
WS=/home/rokey/dev_ws/spot_ws
PKG="$WS/src/smart_farm_spot"
ISAACLAB=/home/rokey/dev_ws/isaac_sim/IsaacLab
VENV=/home/rokey/dev_ws/venv/isaaclab
DISP="${DISPLAY:-:1}"
NUCLEUS="${SF_NUCLEUS:-omniverse://192.168.10.44/Projects/team1/environment}"

echo "════════════════════════════════════════════════════════════"
echo " [서버 모드]  ROS_DOMAIN_ID=$DOMAIN"
echo "   에셋 = Nucleus($NUCLEUS)"
echo "   Nav2 = 서버 것 사용 (여기선 안 띄움)"
echo "════════════════════════════════════════════════════════════"

# 0) Nucleus 도달 확인
if ! timeout 2 bash -c "echo > /dev/tcp/192.168.10.44/3009" 2>/dev/null; then
  echo "  ⚠️ Nucleus(192.168.10.44:3009) 미응답 — 서버 켜졌는지 확인"
fi

# 1) 브릿지/YOLO 만 정리 (서버 Nav2 는 건드리지 않음)
echo "[1/3] 기존 브릿지/YOLO 정리..."
pkill -9 -f "nav_bridge.py" 2>/dev/null || true
pkill -9 -f "yolo_view.py" 2>/dev/null || true
sleep 3

source /opt/ros/humble/setup.bash
[ -f "$WS/install/setup.bash" ] && source "$WS/install/setup.bash"
export ROS_DOMAIN_ID=$DOMAIN

# 2) Isaac 브릿지 (Nucleus 에셋)
echo "[2/3] Isaac 브릿지 실행 (Nucleus 에셋)... Isaac 로딩 ~60s"
rm -f /tmp/navlight.log
PYTHONUNBUFFERED=1 TERM=xterm DISPLAY="$DISP" ROS_DOMAIN_ID=$DOMAIN SF_DOMAIN=$DOMAIN \
  SF_ASSET_BASE="$NUCLEUS" SF_STAND_Z="${SF_STAND_Z:-0.90}" \
  SF_RS_WORLD="${SF_RS_WORLD:-0.0,0.0,0.06}" SF_COW_YAW="${SF_COW_YAW:-0}" \
  setsid bash -c "cd $ISAACLAB && source $VENV/bin/activate && exec ./isaaclab.sh -p $PKG/isaac/nav_bridge.py" \
  > /tmp/navlight.log 2>&1 < /dev/null &
echo "        브릿지 pid $! → /tmp/navlight.log"

# 3) 브릿지 ready 대기 후 YOLO
echo "[3/3] 브릿지 ready 대기..."
until grep -qE "OmniGraph: |Traceback|Killed|Error: " /tmp/navlight.log 2>/dev/null; do sleep 5; done
if grep -qE "Traceback|Killed|Error: " /tmp/navlight.log; then
  echo "  ❌ 브릿지 기동 실패 — /tmp/navlight.log 확인 (Nucleus 접속/경로 확인)"; exit 1
fi
echo "  ✅ 브릿지 ready. YOLO 검출 시작."
setsid python3 "$PKG/yolo_view.py" > /tmp/yolo.log 2>&1 < /dev/null &
echo "        YOLO pid $! → /tmp/yolo.log"

echo ""
echo "✅ 서버 모드 가동 완료 (Nav2 는 팀 서버 것 사용)."
echo "   주행:  ROS_DOMAIN_ID=$DOMAIN python3 $PKG/nav_to_cow.py"
