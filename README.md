# 🐄 spot_ws — 꼬마 로봇 두리 통합 워크스페이스

스마트 축사 자율순찰 Spot 로봇의 **지금 필요한 파일만** 모은 단일 워크스페이스.
흩어져 있던 `smart_farm_spot`(인지) + `spot_nav2`(주행)를 하나로 통합한 결과물.

> 원본은 `~/dev_ws/isaac_sim/smart_farm_spot`, `~/dev_ws/isaac_sim/spot_nav2` 에 그대로 있고
> 백업은 `~/dev_ws/backups/*.tar.gz` 에 있습니다. (검증 후 원본 삭제 가능)

---

## 📂 무엇이 무엇인지 (한눈에)

```
spot_ws/
└── src/
    └── smart_farm_spot/              ← colcon 빌드 대상 ROS 2 패키지
        │
        ├── package.xml / setup.py / setup.cfg / resource/   # ROS 2 패키지 메타
        ├── README.md                                        # 패키지 상세 설명
        │
        ├── smart_farm_spot/          # [ros2 run] ROS 2 노드
        │   └── waypoint_patrol.py    #   뒤쪽 통로 웨이포인트 순찰
        │
        ├── isaac/                    # [isaaclab.sh -p] Isaac Sim 안에서 실행 (ros2 run 아님)
        │   ├── scene_setup.py        #   ★시나리오 배치★ 축사 로드 + 로봇 시작위치 배치
        │   ├── isaac_sim_bridge.py   #   ★통합 브릿지★ 센서 + odom/cmd_vel (Nav2 폐루프)
        │   ├── view_scene.py         #   USD만 띄워서 보는 최소 뷰어
        │   ├── capture_check.py      #   헤드리스 점검 + 스크린샷
        │   └── setup_semantics.py    #   소 객체 시맨틱 라벨(detections용)
        │
        ├── launch/
        │   ├── bringup.launch.py     #   ★통합 실행★ Nav2 + (순찰 옵션)
        │   ├── spot_nav2.launch.py   #   Nav2 스택만
        │   └── spot_slam.launch.py   #   SLAM 매핑
        │
        ├── config/
        │   ├── nav2_params.yaml      #   Nav2 파라미터 (축사 튜닝)
        │   └── waypoints.yaml        #   순찰 웨이포인트
        │
        ├── maps/
        │   ├── environment_0609.yaml #   ★준비한 축사 맵★ (AMCL 기본 맵으로 연결됨)
        │   ├── environment_0609.png  #   점유격자 이미지
        │   └── README.md
        │
        ├── docs/                     # 상세 가이드 (README/INTEGRATION/브릿지/RUN/ASSETS)
        │
        └── wip/
            └── thermal_processor.py  # ⏸ 열화상 비전 준비중 (빌드 제외, 완료 시 모듈로 이동)
```

### 두 가지 실행 주체 (중요)
| 위치 | 실행 방법 | 비고 |
|------|-----------|------|
| `smart_farm_spot/*.py`, `launch/`, `config/` | `ros2 run` / `ros2 launch` | 일반 ROS 2 (venv 불필요) |
| `isaac/*.py` | `isaaclab.sh -p` (venv 활성화) | Isaac Sim SimulationApp 필요 |

### 지금 빠진(=아직 안 쓰는) 것
- **RL 학습 트랙** (`config/spot`, `sensors/`, `assets/`, `mockup/`) → 원본 `isaac_sim/smart_farm_spot/` 에 남겨둠. 주행/시뮬과 별개 트랙이라 이 워크스페이스엔 미포함.
- **열화상 비전** → `wip/` 에 보관, 빌드 제외.

---

## 🛠 빌드

```bash
cd ~/dev_ws/spot_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select smart_farm_spot
source install/setup.bash
```

---

## 🚀 지금 할 수 있는 것 (중간 점검)

### A. 배치 눈으로 보기 (Isaac Sim GUI)
```bash
source ~/dev_ws/venv/isaaclab/bin/activate
cd ~/dev_ws/isaac_sim/IsaacLab
./isaaclab.sh -p ~/dev_ws/spot_ws/src/smart_farm_spot/isaac/scene_setup.py
# 로봇 뒤집힘 → BASE_ROT_WXYZ, 위치 → START_POS 조정
```

### B. Nav2 주행 (3 터미널)
```bash
# 1) Isaac Sim 통합 브릿지
./isaaclab.sh -p ~/dev_ws/spot_ws/src/smart_farm_spot/isaac/isaac_sim_bridge.py \
    --usd /home/rokey/Documents/environment_0609.usd --robot-prim /World/Spot --mode kinematic
# 2) Nav2 (이 축사 맵으로 자동 실행)
ros2 launch smart_farm_spot bringup.launch.py
# 3) 순찰
ros2 run smart_farm_spot waypoint_patrol --ros-args -p loop:=true
```

---

## 🧹 정리 (검증 후 선택)
이 워크스페이스가 잘 돌면 아래는 삭제 가능 (백업 있음):
- `~/dev_ws/isaac_sim/spot_nav2/` 및 `spot_nav2.zip`
- `~/dev_ws/isaac_sim/smart_farm_spot/ros2_bridge/` (브릿지는 isaac/로 병합됨)
