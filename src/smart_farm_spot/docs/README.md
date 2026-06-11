# 🐄 꼬마 로봇 두리 - Smart Farm Spot 시뮬레이션 패키지

> Boston Dynamics **Spot + Arm** 기반 자율순찰 로봇의  
> 유방염(Mastitis) · 파행(Lameness) 조기 탐지 Isaac Lab 환경

---

## 📁 폴더 구조

```
smart_farm_spot/
├── assets/
│   ├── spot_with_arm_sensors.usd     ← 센서 탑재 완료 USD (바로 사용)
│   └── build_sensors_usd.py          ← USD 센서 재빌드 스크립트
│
├── config/
│   └── spot/
│       ├── __init__.py               ← gym 환경 등록
│       ├── spot_arm.py               ← ArticulationCfg (로봇 설정)
│       ├── rough_arm_env_cfg.py      ← 씬/관측/보상/이벤트 전체 설정
│       └── agents/
│           ├── __init__.py
│           └── rsl_rl_ppo_cfg.py     ← PPO 하이퍼파라미터
│
├── sensors/
│   ├── spot_arm_sensor_cfg.py        ← Isaac Lab 센서 CFG
│   └── setup_spot_sensors.py        ← Isaac Sim Script Editor 실행 스크립트
│
├── ros2_bridge/
│   └── thermal_processor.py         ← 열화상 후처리 ROS 2 노드
│
└── docs/
    └── README.md                    ← 이 파일
```

---

## ⚙️ 센서 구성

| 센서 | USD 경로 | 규격 | 용도 |
|------|----------|------|------|
| **RGB Camera** | `/spot/arm0_link_fngr/RGBCamera` | 1280×720 @ 30FPS · 90° FOV | 파행 탐지 + Lock-on 추적 |
| **Thermal Camera** | `/spot/arm0_link_fngr/ThermalCamera` | 640×512 @ 15FPS · 45° FOV | 유방염 열점(40~42°C) |
| **LiDAR 360°** | `/spot/base/Lidar360/lidar_sensor` | VLP-16 · 16ch · 30m | Nav2 충돌 회피 + SLAM |
| **IMU** | `/spot/base/IMU` | 200Hz | RL 짐벌 흔들림 보정 |

두 카메라는 `arm0_link_fngr`(end-effector)에 하향 장착되어,  
로봇팔을 **Downward** 자세로 내리면 소의 유방·다리를 정밀 촬영합니다.

---

## 🚀 빠른 시작

### 전제 조건

```bash
# 환경 확인
isaac-lab 버전: 5.1
rsl_rl:        최신
Python:        3.10
VRAM:          16GB (RTX 5080)
OS:            Ubuntu 22.04
```

### 1단계 - 패키지 설치

```bash
cd ~/dev_ws/isaac_sim/IsaacLab

# config/spot 폴더를 IsaacLab 태스크 경로에 연결
ln -s ~/dev_ws/isaac_sim/smart_farm_spot/config/spot \
    source/extensions/omni.isaac.lab_tasks/omni/isaac/lab_tasks/manager_based/locomotion/velocity/config/spot_arm

# 환경 재등록 (IsaacLab pip 재설치)
pip install -e source/extensions/omni.isaac.lab_tasks --break-system-packages
```

### 2단계 - USD 확인 (선택)

센서가 이미 포함된 `assets/spot_with_arm_sensors.usd`를 그대로 사용합니다.  
원본 USD를 다시 빌드하려면:

```bash
pip install usd-core
python3 assets/build_sensors_usd.py \
    --src /home/rokey/dev_ws/isaac_sim/spot_with_arm.usd \
    --dst assets/spot_with_arm_sensors.usd
```

### 3단계 - USD 경로 확인

`config/spot/spot_arm.py` 상단의 경로를 실제 위치에 맞게 수정합니다.

```python
# config/spot/spot_arm.py  (5번째 줄 근처)
SPOT_ARM_USD_PATH = os.path.join(
    _CURR_DIR, "../../assets/spot_with_arm_sensors.usd"
)
# → 실제 경로 예시:
# SPOT_ARM_USD_PATH = "/home/rokey/dev_ws/isaac_sim/smart_farm_spot/assets/spot_with_arm_sensors.usd"
```

### 4단계 - 학습 실행

```bash
cd ~/dev_ws/isaac_sim/IsaacLab
source /home/rokey/dev_ws/venv/isaaclab/bin/activate

python scripts/reinforcement_learning/rsl_rl/train.py \
    --task Isaac-Velocity-Rough-Spot-Arm-v0 \
    --num_envs 4096 \
    --headless
```

### 5단계 - 시각화 확인 (학습 중)

```bash
# 별도 터미널에서
tensorboard --logdir ~/dev_ws/isaac_sim/IsaacLab/logs/rsl_rl/spot_arm_rough
```

### 6단계 - 추론 실행

```bash
python scripts/reinforcement_learning/rsl_rl/play.py \
    --task Isaac-Velocity-Rough-Spot-Arm-Play-v0 \
    --num_envs 32 \
    --load_run spot_arm_rough \
    --checkpoint model_3000.pt
```

---

## 🔍 Isaac Sim 센서 활성화 (ROS 2 Bridge)

Isaac Sim GUI에서 USD를 열고 센서를 ROS 2 토픽으로 연결하려면:

1. Isaac Sim 실행 후 `assets/spot_with_arm_sensors.usd` 열기
2. **Window > Extensions** → `omni.isaac.ros2_bridge` 활성화
3. **Window > Script Editor** 열기
4. `sensors/setup_spot_sensors.py` 내용 붙여넣기 → **실행(▶)**
5. Play(▶) 버튼 → ROS 2 토픽 발행 시작

```bash
# 토픽 확인
ros2 topic list | grep spot
# 출력:
# /spot/arm/rgb/image_raw
# /spot/arm/thermal/image_raw
# /spot/scan
# /spot/imu/data
# /tf
```

---

## 🌡️ 열화상 후처리

Isaac Sim은 네이티브 열화상 카메라를 미지원합니다.  
대신 소 유방 머티리얼에 **Thermal Emission** 속성을 부여하고,  
RGB 영상을 JET colormap으로 후처리하여 열화상을 모사합니다.

```bash
# 열화상 후처리 노드 실행 (ROS 2 Humble 환경)
source /opt/ros/humble/setup.bash
python3 ros2_bridge/thermal_processor.py

# 열화상 영상 확인
ros2 run rqt_image_view rqt_image_view /spot/arm/thermal/colormap
```

유방염 탐지 결과는 `/spot/arm/thermal/colormap` 토픽에서  
**주황색 박스(열점 Bounding Box)** 로 오버레이되어 출력됩니다.

---

## 🔧 주요 파라미터 조정

### 스폰 높이 (CoM 안정성)

```python
# config/spot/spot_arm.py
init_state=ArticulationCfg.InitialStateCfg(
    pos=(0.0, 0.0, 0.65),   # 기본값: 0.65m
    # z가 너무 낮으면 다리가 바닥을 치며 반발 → 뒤로 넘어짐
    # z가 너무 높으면 낙하 충격 → 0.60~0.70m 사이 조정
)
```

### 팔 초기 자세 (CoM 전후 조정)

```python
INIT_JOINT_POS = {
    "arm0_sh1": -2.6,   # -3.054(완전 접힘) ~ -2.0(앞으로 뻗음)
    # 값이 클수록(0에 가까울수록) end-effector가 앞쪽으로 이동 → CoM 후방 이탈 방지
}
```

### 환경 수 (VRAM 조정)

```python
# rough_arm_env_cfg.py
scene: SpotArmSceneCfg = SpotArmSceneCfg(
    num_envs=4096,    # RTX 5080 16GB 기준 최대
    # VRAM 부족 시: 2048 또는 1024로 낮추기
)
```

---

## 🐛 자주 발생하는 문제

### USD 로드 시 cycle 경고
```
Warning: Sublayer hierarchy ... has cycles.
```
→ `build_sensors_usd.py`가 자동으로 self-reference를 제거합니다.  
   이미 생성된 `spot_with_arm_sensors.usd`는 문제없습니다.

### `AttributeError: 'ArticulationCfg' has no attribute 'default_joint_pos'`
→ `spot_arm.py`에서 `SPOT_ARM_CFG.default_joint_pos = ...` 라인이 있다면 **삭제**하세요.  
   조인트 초기값은 `init_state.joint_pos`로만 설정합니다.

### `ImportError: cannot import name 'fold_arm_over_time'`
→ `__init__.py`에서 fold 관련 gym 등록 항목이 남아있다면 삭제하세요.  
   현재 `__init__.py`에는 fold 관련 항목이 없습니다.

### `height_scanner` prim 경로 오류
```
[Error] Prim path ... does not exist
```
→ `rough_arm_env_cfg.py`의 height_scanner prim_path가  
   `{ENV_REGEX_NS}/Robot/base`로 되어 있는지 확인하세요.

### ROS 2 토픽 미발행
→ Isaac Sim > Window > Extensions에서 `omni.isaac.ros2_bridge` 활성화 여부 확인  
→ Play(▶) 버튼을 누른 상태에서만 토픽이 발행됩니다.

---

## 📊 학습 진행 기준 (RTX 5080, 4096 env)

| 단계 | Iteration | 예상 시간 | 확인 지표 |
|------|-----------|-----------|-----------|
| 초기 기립 | ~200 | ~40분 | 넘어지지 않고 서 있음 |
| 기본 보행 | ~500 | ~1.5시간 | 앞으로 직진 가능 |
| 거친 지형 | ~1500 | ~5시간 | 장애물 통과 가능 |
| 안정 보행 | ~3000 | ~12시간 | 목표 속도 추종 |

---

## 📡 ROS 2 토픽 요약

| 토픽 | 타입 | 발행 주기 | 발행 노드 |
|------|------|-----------|-----------|
| `/spot/arm/rgb/image_raw` | `sensor_msgs/Image` | 30Hz | Isaac Sim OmniGraph |
| `/spot/arm/thermal/image_raw` | `sensor_msgs/Image` | 15Hz | Isaac Sim OmniGraph |
| `/spot/arm/thermal/colormap` | `sensor_msgs/Image` | 15Hz | thermal_processor.py |
| `/spot/scan` | `sensor_msgs/LaserScan` | 10Hz | Isaac Sim OmniGraph |
| `/spot/imu/data` | `sensor_msgs/Imu` | 200Hz | Isaac Sim OmniGraph |
| `/tf` | `tf2_msgs/TFMessage` | 30Hz | Isaac Sim OmniGraph |

---

## 👥 팀 역할 연계

| 파일 | 담당자 | 비고 |
|------|--------|------|
| `spot_arm.py`, `rough_arm_env_cfg.py` | 26L (Claude 협업) | RL 보행 학습 |
| `setup_spot_sensors.py` | 김승우 (PM) | ROS2 브리지 |
| `assets/spot_with_arm_sensors.usd` | 김현진 | Isaac Sim 씬 |
| `thermal_processor.py` | 성형석 | Vision AI 전처리 |
| 대시보드 연동 | 김도연 | WebRTC → Cloud |
