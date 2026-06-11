# 🐄 smart_farm_spot — 꼬마 로봇 두리 통합 패키지

> Boston Dynamics **Spot + Arm** 기반 스마트 축사 자율순찰 로봇.
> **Isaac Sim 인지**(카메라·열화상·LiDAR·IMU·객체인식) + **Nav2 자율주행**(웨이포인트 순찰·충돌 회피)을
> 하나의 ROS 2 패키지로 통합.
>
> 기존 `smart_farm_spot`(인지/RL) + `spot_nav2`(주행) → **smart_farm_spot v2.0** 통합본

---

## 📁 구조

```
smart_farm_spot/
├── package.xml / setup.py / setup.cfg     # ROS 2 ament_python 패키지
├── resource/smart_farm_spot               # ament 마커
│
├── smart_farm_spot/                        # [ros2 run] ROS 2 노드
│   └── waypoint_patrol.py                  #   뒤쪽 통로 웨이포인트 순찰
│   #  (thermal_processor 열화상 후처리는 비전 준비중 → 제외,
│   #   원본은 ros2_bridge/thermal_processor.py 에 보관)
│
├── isaac/                                  # [python.sh] Isaac Sim 내부 실행
│   ├── isaac_sim_bridge.py                 #   ★통합 브릿지(센서 + odom/cmd_vel)★
│   └── setup_semantics.py                  #   객체 인식 시맨틱 라벨링
│
├── launch/
│   ├── bringup.launch.py                   #   ★통합 bringup (Nav2 + 열화상 + 순찰)★
│   ├── spot_nav2.launch.py                 #   Nav2 스택만
│   └── spot_slam.launch.py                 #   SLAM 매핑
│
├── config/
│   ├── nav2_params.yaml                    #   Nav2 파라미터 (축사 튜닝)
│   ├── waypoints.yaml                      #   순찰 웨이포인트
│   └── spot/                               #   [Isaac Lab] RL 학습 설정 (ros2 빌드 제외)
│
├── maps/                                   #   AMCL 사전 맵
├── sensors/ · assets/ · mockup/            #   [Isaac Lab/Sim] 센서·USD·목업 (참조)
└── docs/                                   #   상세 가이드
```

> **두 종류의 실행 주체**
> - `smart_farm_spot/*.py` → `ros2 run` / `ros2 launch` (일반 ROS 2)
> - `isaac/*.py` → `~/isaacsim/python.sh` (Isaac Sim SimulationApp 필요, `ros2 run` 불가)

---

## 🔗 데이터 흐름 (통합)

```
[Isaac Sim]  isaac/isaac_sim_bridge.py            [ROS 2]
  ┌─ 센서 OmniGraph ───────────────────────────────────────────────┐
  │  RGB 카메라 ─┬─→ /spot/arm/rgb/image_raw ──────→ (클라우드 촬영본)
  │             └─→ /spot/arm/detections ─────────→ RL 짐벌 Lock-on
  │  Thermal ───→ (비전 준비중 — 통합 제외, ENABLE_THERMAL=False)
  │  LiDAR ─────→ /spot/scan ─────────────────────→ Nav2 Costmap
  │  IMU ───────→ /spot/imu/data ─────────────────→ 보행/짐벌
  └─ Nav 브릿지 ───────────────────────────────────────────────────┘
     /odom + odom→base_link TF ──────────────────→ AMCL / TF
     /cmd_vel 수신 ←──────────────────────────────── velocity_smoother ← DWB
        (베이스 구동)                                       ↑
                                                     waypoint_patrol
```

핵심: 두 브릿지를 **하나의 `isaac_sim_bridge.py`** 로 병합 → Isaac Sim 한 번 실행으로
센서 발행 + Nav2 폐루프(cmd_vel↔odom)가 동시에 동작. TF 충돌 방지를 위해 센서
OmniGraph의 TF 발행은 끄고 Nav 브릿지가 TF를 전담.

---

## 🛠 빌드

```bash
# 1. 워크스페이스 src에 배치 (심볼릭 링크 또는 복사)
cd ~/dev_ws/edge_robot_ws/src
ln -s ~/dev_ws/isaac_sim/smart_farm_spot .

# 2. 빌드
cd ~/dev_ws/edge_robot_ws
colcon build --packages-select smart_farm_spot
source install/setup.bash
```

### 의존성
```bash
sudo apt install -y \
    ros-humble-navigation2 ros-humble-nav2-bringup \
    ros-humble-slam-toolbox ros-humble-teleop-twist-keyboard \
    ros-humble-vision-msgs
# 열화상(thermal_processor) 비전 작업 시 추가: ros-humble-cv-bridge python3-opencv
```

---

## 🚀 실행 (3 터미널)

### 터미널 1 — Isaac Sim 통합 브릿지
```bash
~/isaacsim/python.sh \
    $(ros2 pkg prefix smart_farm_spot)/share/smart_farm_spot/isaac/isaac_sim_bridge.py \
    --usd /path/to/barn_scene.usd \
    --robot-prim /World/Spot \
    --mode kinematic
# 옵션: --no-sensors (Nav만)  --gui  --no-semantics
```

### 터미널 2 — ROS 2 통합 bringup (Nav2)
```bash
source ~/dev_ws/edge_robot_ws/install/setup.bash
ros2 launch smart_farm_spot bringup.launch.py

# 맵 없이 빠른 테스트
ros2 launch smart_farm_spot bringup.launch.py use_amcl:=false
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom
```

초기 위치 설정 (AMCL 사용 시):
```bash
ros2 topic pub --once /initialpose geometry_msgs/PoseWithCovarianceStamped \
  "{header: {frame_id: map}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"
```

### 터미널 3 — 순찰
```bash
source ~/dev_ws/edge_robot_ws/install/setup.bash
ros2 run smart_farm_spot waypoint_patrol                       # 1회
ros2 run smart_farm_spot waypoint_patrol --ros-args -p loop:=true   # 무한
```

> `bringup.launch.py patrol:=true loop:=true` 로 순찰까지 한 번에 띄울 수도 있습니다.

---

## 🧠 RL 학습 (Isaac Lab — 참조)

`config/spot/`, `sensors/`, `assets/` 는 Isaac Lab 보행 학습용으로, ROS 2 빌드에는
포함되지 않습니다. 학습 절차는 [docs/README.md](docs/README.md) 참고.

---

## 📚 문서

| 문서 | 내용 |
|------|------|
| [docs/README.md](docs/README.md) | RL 학습/센서 구성 전체 가이드 |
| [docs/ROS2_BRIDGE_GUIDE.md](docs/ROS2_BRIDGE_GUIDE.md) | 센서 브릿지/객체인식 상세 |
| [docs/INTEGRATION.md](docs/INTEGRATION.md) | spot_nav2 통합 내역/매핑 |
| [maps/README.md](maps/README.md) | 맵 생성 방법 |
```
