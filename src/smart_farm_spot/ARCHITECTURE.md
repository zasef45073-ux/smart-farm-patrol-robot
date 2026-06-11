# 스마트팜 순찰 로봇 "꼬마 두리" — 아키텍처 / 진행 현황

축사 순찰 로봇(Boston Dynamics Spot + 팔). **순찰 → 소 검출 → 소 후방(꼬리쪽) 이동 → (검사)** 시나리오를
Isaac Sim 5.1 + ROS2 Humble 에서 **SLAM + Nav2 + RL 보행** 구조로 구현.

---

## 1. 시스템 아키텍처

```
        ┌─────────────────────────── Isaac Sim (venv python3.11) ───────────────────────────┐
        │  scenario.py  (RL 브릿지)                                                          │
        │   · 평면 + 안정 평면정책(spot_flat, 235차원·다리12관절) → 보행                      │
        │   · /cmd_vel 구독 → vel_command_b 주입 (정책이 그대로 추종)                         │
        │   · 축사(구역마찰·가상벽) + 텍스처 소 + 팔(검사자세) + RTX 라이다 + 손 RealSense    │
        │   · 발행: /scan /odom /tf(odom→base_link, base→lidar) /clock                       │
        │           /spot_cam/{rgb,depth,camera_info}(320×320)                               │
        │   · 넘어짐 복구(강제 기립) · 대기 중 완전정지(Nav2 명령 전까지)                     │
        └───────────────┬──────────────────────────────────────────┬────────────────────────┘
                        │ /scan /odom /tf                           │ /spot_cam/*
                        ▼                                           ▼
        ┌──────────────────────────┐   /map+map→odom   ┌────────────────────────────┐
        │ slam_toolbox (SLAM)      │ ────────────────► │ yolo_view.py (Pose 검출)   │
        │  /scan → /map, map→odom  │                   │  /spot_cam/rgb+depth →     │
        └──────────────┬───────────┘                   │  YOLO(best.pt) → 소+키포인트 │
                       │ /map                            │  + 거리 → /yolo/annotated  │
                       ▼                                 └────────────────────────────┘
        ┌──────────────────────────────────────┐
        │ Nav2 (경로계획·장애물회피)            │   /cmd_vel
        │  global/local costmap(/scan) + DWB    │ ─────────────►  (Isaac scenario.py 로 주입)
        │  · 후진 금지(min_vel_x=0) → 전진형    │
        │  · NavfnPlanner(A*) / DWBLocalPlanner │
        └──────────────▲───────────────────────┘
                       │ NavigateToPose 목표
        ┌──────────────┴───────────────────────┐
        │ 목표 송신 노드 (택1)                  │
        │  · scenario_nav.py : 소 후방 1지점    │   ← /tmp/scenario_goal.json (소 후방 좌표)
        │  · patrol.py       : 통로 끝 전부 순회 │   ← /tmp/patrol_waypoints.json (통로 끝들)
        └───────────────────────────────────────┘
```

- **SLAM**: `slam_toolbox` — `/scan` 으로 `/map` 생성 + `map→odom` 보정.
- **Nav2**: 코스트맵 경로계획·장애물회피 → `/cmd_vel`. **후진 금지**(DWB `min_vel_x=0`)로 목표를 향해 돌아서 전진.
- **RL**: 평면정책이 `/cmd_vel` 을 받아 다리 12관절 보행(정책 파일 무수정).
- **인지**: 손 RealSense(팔 끝) → YOLO Pose 로 소 검출 + 14 키포인트 + 뎁스 거리.

도메인: **ROS_DOMAIN_ID=153** (모든 노드 명시).

---

## 2. 구성 파일

| 파일 | 역할 |
|---|---|
| `isaac/scenario.py` | **RL 브릿지** — 정책 보행 + /cmd_vel 구독 + 축사/소/팔/라이다/카메라 + ROS 발행 + 복구/정지 |
| `isaac/isaac_sim_bridge.py` | **통합 브릿지** — Nav2(odom/cmd_vel) 및 센서 통합 ROS 2 브릿지 |
| `isaac/scene_setup.py` | 축사 환경 로드 및 로봇 정밀 초기 배치 |
| `isaac/setup_semantics.py` | 소 객체 시맨틱 라벨링(Ground-Truth Bounding Box 생성) |
| `isaac/drive_course.py` | H자(또는 임의) 코스 Kinematic 주행(Nav2 없이 직접 cmd_vel 적용) 데모 |
| `isaac/camera_record.py` | 헤드리스 손끝 RealSense (RGB+Depth) 카메라 녹화 및 ROS bag 발행 |
| `patrol.py` | **순찰** — 통로 끝 웨이포인트를 SLAM+Nav2+RL 로 순서대로 순회(반복) |
| `scenario_nav.py` | 소 후방(꼬리 1.5m 뒤) 1지점으로 NavigateToPose |
| `yolo_view.py` | YOLO Pose 소 검출 + 거리 → `/yolo/annotated`, 파행 비대칭 지표 → `/cow/lameness` |
| `scripts/run_scenario.sh` | 전체 오케스트레이션(좀비정리→브릿지→SLAM→Nav2→YOLO→목표/순찰). `SF_PATROL=1`=순찰 |
| `config/nav2_params_slam.yaml` | Nav2 파라미터(후진금지, inflation 0.28, DWB, NavfnPlanner) |
| `config/waypoints_h.yaml` | H자 주행코스 웨이포인트 목록 |
| `config/waypoints_diag.yaml` | 대각선 주행코스 웨이포인트 목록 |
| `config/scenario.rviz` | RViz(odom fixed, TF/Odometry/LaserScan/Image) |
| `launch/bringup.launch.py` | **신규 통합 런치** — 브릿지, Nav2, 시나리오 노드 등 일괄 실행 |
| `launch/spot_nav2.launch.py` | 패키지명이 갱신된 기존 Nav2 런치 |
| `launch/spot_slam.launch.py` | slam_toolbox (scan_topic /scan) |

정책: `logs/rsl_rl/spot_flat/2026-06-07_14-01-53/exported/policy.pt` (**무수정** — 안정 평면정책).

---

## 3. 완성된 부분 ✅

- **안정 보행**: 평면 + spot_flat 정책 → 넘어짐 거의 없음, 넘어지면 강제 기립 복구.
- **축사 정렬**: 바닥 z = 발 접지 정렬(FLOOR_Z=feet−0.20), 구역별 마찰(우리 0.45/통로 0.7), 가상벽/펜스 콜라이더.
- **소**: cow_eat.usd + 비균일 스케일(2.37×1.69×1.69) + T_Cow_B 텍스처(st UV) + yaw(반대 방향=뒤태 노출).
- **센서**: RTX 라이다 → /scan, 손 RealSense RGB+Depth 320×320 → /spot_cam/*, camera_info.
- **SLAM + Nav2 + RL 주행**: 후진금지 전진형으로 목표까지 헤딩 맞춰 이동.
- **소 후방 이동**: 소 bbox로 꼬리끝 계산 → 1.5m 뒤 목표 → 도착("★ 소 후방 도착! 검사 자세").
- **YOLO 검출**: /spot_cam 토픽 연결 + best.pt(Pose, 14키포인트) → 소 검출(접근 시 conf↑, 1.28m서 0.86) + 거리.
- **순찰(patrol.py)**: 통로 끝 6개 자동추출(H코스 ±3.3, ±16.3×±7) → 순서대로 순회.
- **대기 중 완전정지**: Nav2 명령(/cmd_vel≠0) 전까지 제자리 → 초기 배회/드리프트 방지.
- **시작점=통로 끝**: 선택 통로 끝이 로봇 스폰에 오도록 축사 이동(중앙 출발 X). `SF_START_AT_END`.
- **디버그 3인칭 시점**: 기동 시 축사 전체 조망 고정(`SF_DBG_EYE_*`).

---

## 4. 주요 환경변수

| 변수 | 기본 | 의미 |
|---|---|---|
| `SF_POLICY` | (assets) | 정책 경로 (run_scenario.sh 가 spot_flat 지정) |
| `SF_START_AT_END` | 1 | 시작점=통로 끝(중앙 X). 0=중앙 |
| `SF_START_END_IDX` | 0 | 시작 통로 끝 선택(먼 끝부터) |
| `SF_PATROL` | 0 | 1=순찰(patrol.py), 0=소 후방 1지점(scenario_nav.py) |
| `SF_OPEN_SH1` | -0.3 | 팔 어깨각(−=들림, creep 방지) |
| `SF_OPEN_WR0` | 0.6 | 손목각(카메라 아래로 조준) |
| `SF_START_STOP_SECS` | 90 | 대기 정지 안전 상한(실제론 Nav2 명령에 즉시 출발) |
| `SF_COW_DZ` | -0.15 | 소 z 내림 |
| `SF_PEN/CORRIDOR_FRICTION` | 0.45/0.7 | 구역 마찰 |
| `SF_DBG_EYE_X/Y/Z` | 8/1/8 | 디버그 3인칭 시점 위치 |

---

## 5. 실행

```bash
# 순찰(통로 끝 전부)
SF_PATROL=1 bash src/smart_farm_spot/scripts/run_scenario.sh
# 소 후방 이동 시나리오
bash src/smart_farm_spot/scripts/run_scenario.sh
# 검출 화면
ROS_DOMAIN_ID=153 rqt_image_view /yolo/annotated
```

---

## 6. 보류 / 향후 (TODO)

- **등쪽 팔 수납**: 다리전용 정책은 팔을 관측 안 해 머리 위로 넘기면 무게중심 쏠려 뒤집힘.
  → **팔 관측 포함 SpotArm 정책 재학습 후** 적용(코드는 주석으로 보존).
- **측정기반 목표**: 현재 소 후방 목표는 시뮬 정답좌표 사용. → YOLO+뎁스+TF 로 **검출값 기반** 목표 산출로 교체 필요
  (전제: `odom→spot_cam` 동적 TF 발행 = 손링크 FK).
- **파행(lameness) 검사**: 모델은 클래스 'cow' 1개 + 14키포인트만 → 파행 라벨 없음.
  → ✅ **1차 휴리스틱 구현**: `yolo_view.py` 가 키포인트를 bbox 세로 중심선 기준으로 좌우
     미러링해 비대칭 지표(0~1)를 산출, 소별로 `/cow/lameness`(Float32MultiArray) 발행 +
     오버레이 표시(`limp 0.xx`, 임계 초과 시 `LAME?`). 키포인트 의미순서에 비의존.
  → 향후: 파행 라벨 데이터로 모델 재학습 + 다프레임(보행주기) 시계열 지표로 정밀화.
- **검출 신뢰도 향상**: 320×320 저해상도 + Sim-to-Real 도메인 갭 + 뒤태/스케일.
  → 도메인 랜덤화(소 scale·pose·조명·카메라) + 시뮬 자동 bbox 라벨 → best.pt 파인튜닝.
```
