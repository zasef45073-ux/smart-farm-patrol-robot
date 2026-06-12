# 스마트팜 순찰 로봇 "꼬마 두리" — 아키텍처

축사 순찰 로봇(Boston Dynamics Spot + 팔). **순찰 → 소 검출 → 소 후방(꼬리쪽) 접근 → 검사** 시나리오를
**Isaac Sim 5.1 + ROS 2 Humble** 에서 구현. 도메인 **`ROS_DOMAIN_ID=153`**(모든 노드 명시).

---

## 1. 3계층 구조 (역할 분담)

| 계층 | 담당 | 구현 |
|------|------|------|
| **RL 보행** | 저수준 다리 12관절 제어 | `spot_flat` 무팔 평면정책(무수정) — 목표속도(vx,wz)→보행 |
| **위치추정+맵** | localization / mapping | **3가지 모드 선택** (아래 §3) |
| **Nav2** | 경로계획·장애물회피 | costmap(/scan + 사전맵 + keepout) → /cmd_vel |
| **인지** | 소 검출·파행 | 손 RealSense → YOLO-Pose → /yolo/annotated, /cow/lameness |

> **RL ↔ Nav2 분리**: Nav2는 "어디로 얼마 속도로"만 지시(`/cmd_vel`), 실제 다리 보행은 RL이 담당.

---

## 2. 폐루프 데이터 흐름

```
         목표(소 후방 / 순찰 웨이포인트)
              │ NavigateToPose
              ▼
   ┌─────────┐  /cmd_vel(vx,wz)   ┌──────────────────────────┐
   │  Nav2   │ ─────────────────► │ scenario.py (Isaac, venv) │
   │ 경로계획 │                    │  └ RL 정책 → 다리 보행     │
   └─────────┘                    │  + 축사·소·팔·RTX라이다·   │
        ▲                         │    손RealSense             │
        │ /map · /scan 코스트맵    └──────────┬───────────────┘
        │                          /scan /odom /tf /spot_cam
   ┌────┴───────────────┐  map→odom         │
   │ 위치추정/맵 (택1)   │ ◀─────────────────┤
   │ §3 의 3모드 중 하나 │                   ▼
   └────────────────────┘            ┌──────────────────────┐
                                     │ yolo_view.py          │
                                     │ /spot_cam → YOLO Pose │
                                     │ → /yolo/annotated     │
                                     │ → /cow/lameness(파행) │
                                     └──────────────────────┘
```

한 사이클: **Nav2 경로 → /cmd_vel → RL 보행 → 로봇 이동 → /scan·/odom → 위치추정/맵 갱신 → Nav2 재계획**.

---

## 3. 위치추정 3가지 모드 ★

`map→odom` TF 는 발행자가 하나여야 하므로 셋 중 **하나만** 실행한다.

| 모드 | 스크립트 | map | map→odom | 특징 |
|------|----------|-----|----------|------|
| **사전맵+정위치** | `run_scenario.sh` | `barn_map_server`(USD 래스터, **투명벽 포함**) | **identity**(시뮬 정위치) | 기본·결정론적. 엄밀히는 SLAM 아님(맵 주어짐) |
| **AMCL** | `run_scenario_amcl.sh` | barn_map_server(`SF_NO_MAP_TF=1`) | **AMCL 파티클필터 추정** | 사전맵 + 위치추정. `nav2_params_amcl.yaml` |
| **SLAM / 복합** | `run_scenario_slam.sh` | `slam_toolbox` 실시간 작성 | scan-match 추정 | `SF_SLAM_MODE=mapping`(순수) / `localization`(map_and_localization=AMCL+SLAM 복합) |

- **왜 기본이 사전맵?** ① 라이다가 못 보는 **투명/가상벽**을 USD에서 직접 넣음 ② 시뮬은 정위치를 알아 드리프트 0 ③ 데모 결정론.
- **keepout 필터**: 세 모드 공통 — 투명벽을 코스트맵 lethal 로 보강(`keepout.launch.py`, 기본 ON). `nav2_params_slam.yaml`이 필수 요구하므로 안 띄우면 코스트맵 정체.

---

## 4. 구성요소

### Isaac Sim 측 (venv python3.11, `isaaclab.sh -p`)
| 파일 | 역할 |
|------|------|
| `isaac/scenario.py` | **RL 브릿지** — 정책 보행 + /cmd_vel 구독 + 축사(구역마찰·가상벽)·소·팔·RTX라이다(2D,360°)·손RealSense. 발행: /scan /odom /tf /clock /spot_cam/{rgb,depth,camera_info}. 넘어짐 복구·시작 정지 |
| `isaac/record_4cam.py` | **헤드리스 4뷰 녹화** — 축사사선/로봇체이스/손RealSense/탑다운맵 → mp4(`SF_OUT_DIR`). 팔경량화·조명감광·노출게인 |
| `isaac/camera_record.py` | 손끝 RealSense(RGB+D) 단일 녹화 + rosbag |
| `isaac/scene_setup.py` / `setup_semantics.py` | 씬 배치 / 소 시맨틱 라벨 |

### ROS 2 측 — `bringup.launch.py` 하나로 통합 (인자 토글)
| 노드/기능 | 인자 | 토픽/역할 |
|-----------|------|-----------|
| Nav2 스택 | (항상) | 경로계획 + 코스트맵 |
| waypoint_patrol | `patrol:=true` | 순찰 |
| twist_mux | `twist_mux:=true` | 다중 cmd_vel 우선순위 중재 + e_stop |
| dashboard_bridge | `dashboard:=true` | 웹 대시보드 ↔ 로봇 |
| keepout | `keepout:=true` | 투명벽 필터 |
| yolo_view | `yolo:=true` | /yolo/annotated, /cow/lameness |

### 행동/오케스트레이션 (스크립트가 호출)
| 파일 | 역할 |
|------|------|
| `scenario_nav.py` | 순찰→소 검출→**소 후방(꼬리 1.5m 뒤)** NavigateToPose relay |
| `patrol.py` | 통로 끝 전부 순회 |
| `cow_tail_seek.py` | 비전 전용 꼬리(kpt5) 후방 접근(`tail_search/cmd_vel`) |
| `h_drive.py` | NavigateToPose 순차 주행(FollowWaypoints 행 회피) |
| `barn_map_server.py` | USD 래스터 사전맵 `/map`(+identity TF, `SF_NO_MAP_TF`로 끔) |
| `dashboard/` | FastAPI+MQTT+SQLite 웹 대시보드(검출·카메라·순찰·비상정지) |

---

## 5. 센서 / 인지

- **RTX 라이다**: `Example_Rotary_2D` (360° 회전, **2D LaserScan**), 마운트 높이 `SF_LIDAR_Z=0.55`, **근거리 한계 1.0m**(nearRange) — 바닥 히트(-2°@0.55m≈15.7m) 회피 위해 `obstacle_max_range=12`.
- **손 RealSense**: arm0_link_fngr RGB+Depth → `/spot_cam/*`.
- **YOLO**: `best.pt`(Pose, cow 1클래스 + 14키포인트) → 소 검출 + 거리 + **파행 휴리스틱**(키포인트 좌우 미러 비대칭 0~1 → `/cow/lameness`).

---

## 6. 실행 (요약)

```bash
# 공통: source /opt/ros/humble/setup.bash; source install/setup.bash; export ROS_DOMAIN_ID=153

# A. 시나리오 (위치추정 모드 택1)
bash src/smart_farm_spot/scripts/run_scenario.sh            # 사전맵+정위치(기본)
bash src/smart_farm_spot/scripts/run_scenario_amcl.sh       # AMCL
SF_SLAM_MODE=mapping bash .../run_scenario_slam.sh          # SLAM
#  옵션: SF_PATROL=1(순찰) · SF_VISION_TAIL=1(비전목표) · SF_KEEPOUT=0(끄기)

# B. ROS2 기능 통합 launch (Isaac 별도)
ros2 launch smart_farm_spot bringup.launch.py \
    patrol:=true twist_mux:=true dashboard:=true keepout:=true yolo:=true

# C. 4뷰 헤드리스 녹화
SF_HEADLESS=1 SF_OUT_DIR=~/rag/demo SF_REC_RES=1280,640 \
    ./isaaclab.sh -p .../isaac/record_4cam.py     # (isaaclab venv)

# 검출 화면
ROS_DOMAIN_ID=153 rqt_image_view /yolo/annotated
```

---

## 7. 핵심 설계 결정

1. **무팔 평면정책 + 팔 경량화/수납** — 다리전용 정책에 팔이 외란. `apply_light_arm`(SF_ARM_LIGHT_KG) 또는 stow 자세로 CoM 중앙화.
2. **후진 금지(DWB min_vel_x=0)** — 목표를 향해 돌아서 전진(4족 자연 보행).
3. **사전맵 기본** — 투명벽 + 정위치 + 결정론(데모 안정). SLAM/AMCL은 변형 제공.
4. **keepout 필수** — 라이다 미감지 투명벽을 코스트맵에 강제(없으면 통과/정체).
5. **2D 라이다** — 평면 축사엔 LaserScan 충분(3D 포인트클라우드 미사용).

---

## 8. 미구현 / 향후 (TODO)

- **측정기반 목표**: 현재 소 후방은 시뮬 정답좌표(`SF_VISION_TAIL=0`). → YOLO+뎁스+TF 검출값 기반으로 교체(전제: odom→spot_cam 동적 TF).
- **무팔 험지(rough) 정책**: 학습본 없음(평지만). IsaacLab `Isaac-Velocity-Rough-Spot` 태스크 주석 해제 + 학습 필요.
- **파행 정밀화**: 1차 휴리스틱 → 파행 라벨 재학습 + 다프레임 시계열.
- **유방 Emission 머티리얼 / scenario 완주(전 소 순회) / 검출 파인튜닝**.
- **검사 단계**: 순찰→검출→후방접근까지 라이브 검증됨, 검사(inspect) 자세 시퀀스는 미실행.

> 상세 다이어그램(flowchart/state/sequence/component)은 루트 `SYSTEM_ARCHITECTURE.md` 참조.
