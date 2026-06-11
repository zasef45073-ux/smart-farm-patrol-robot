# 🚀 실행 가이드 (RUN GUIDE)

> 꼬마 로봇 두리 - Spot + Arm 시뮬레이션  
> **0부터 ROS 2 브릿지 + 객체 인식 동작까지** 단계별 실행 문서

---

## 목차

1. [사전 준비](#1-사전-준비)
2. [파일 배치](#2-파일-배치)
3. [시나리오 A: 지금 당장 테스트 (목업)](#3-시나리오-a-지금-당장-테스트-목업)
4. [시나리오 B: 보행 학습](#4-시나리오-b-보행-학습)
5. [시나리오 C: 실제 소 에셋 연동](#5-시나리오-c-실제-소-에셋-연동)
6. [전체 체크리스트](#6-전체-체크리스트)

---

## 1. 사전 준비

### 환경 확인

```bash
# Isaac Sim 5.1 설치 경로 확인
ls ~/isaacsim/python.sh        # 또는 본인 설치 경로

# Isaac Lab venv 활성화
source /home/rokey/dev_ws/venv/isaaclab/bin/activate

# ROS 2 Humble 확인
source /opt/ros/humble/setup.bash
ros2 --version
```

### 필요 ROS 2 패키지 설치

```bash
sudo apt update
sudo apt install -y \
    ros-humble-vision-msgs \
    ros-humble-rqt-image-view \
    ros-humble-cv-bridge
```

### ROS_DOMAIN_ID 통일 (중요)

Isaac Sim과 터미널이 같은 도메인을 써야 토픽이 보입니다.

```bash
# ~/.bashrc 에 추가
export ROS_DOMAIN_ID=0
```

---

## 2. 파일 배치

압축을 풀고 작업 경로에 배치합니다.

```bash
# 예시 배치 경로
cd ~/dev_ws/isaac_sim/
unzip smart_farm_spot.zip

# 구조 확인
smart_farm_spot/
├── assets/          # USD 에셋
├── config/spot/     # Isaac Lab 학습 설정
├── ros2_bridge/     # ROS 2 브릿지 (핵심)
├── mockup/          # 테스트용 목업 코드
├── sensors/         # 센서 CFG
└── docs/            # 문서
```

---

## 3. 시나리오 A: 지금 당장 테스트 (목업)

> 실제 소 에셋(Day 1)이 없어도 **브릿지 + 객체 인식 + 짐벌 흐름**을 검증합니다.  
> 가짜 소(도형)로 파이프라인 전체를 돌려봅니다.

### A-1. Isaac Sim GUI로 브릿지 + 가짜 소 실행

```
1. Isaac Sim 실행
2. assets/spot_with_arm.usd 열기
   File > Open > spot_with_arm.usd
3. ROS 2 Bridge 익스텐션 확인
   Window > Extensions → "isaacsim.ros2.bridge" 검색 → 활성화(ON)
4. Script Editor 열기
   Window > Script Editor
```

**Script Editor에 순서대로 실행:**

먼저 가짜 소 생성:
```python
import sys
sys.path.insert(0, "/home/rokey/dev_ws/isaac_sim/smart_farm_spot/mockup")
sys.path.insert(0, "/home/rokey/dev_ws/isaac_sim/smart_farm_spot/ros2_bridge")

import mock_scene
mock_scene.build_mock_cow()        # 로봇 앞 1.2m에 가짜 소 생성
```

그 다음 브릿지 구성:
```python
import isaac_ros2_bridge as bridge
bridge.setup_full_bridge(auto_semantics=True)
```

**마지막으로 Play(▶) 버튼 클릭** → 토픽 발행 시작.

### A-2. 토픽 발행 확인 (새 터미널)

```bash
source /opt/ros/humble/setup.bash
cd ~/dev_ws/isaac_sim/smart_farm_spot/mockup

# 전체 토픽 상태 대시보드
python3 mock_subscribers.py
```

정상이면 이렇게 출력됩니다:
```
================================================================
 브릿지 상태 대시보드  (경과 5초)
================================================================
  🟢 RGB 영상     30.0Hz  1280x720 rgb8
  🟢 객체 인식    29.8Hz  2개: udder@(640,360), leg@(580,400)
  🟢 열화상       15.0Hz  640x512 rgb8
  🟢 LiDAR        10.0Hz  360점, 유효 120, 최소 0.85m
  🟢 IMU         200.0Hz  accel=(+0.01,-0.02,+9.81)
```

### A-3. 객체 인식 → 짐벌 Lock-on 흐름 확인

```bash
# 또 다른 터미널
source /opt/ros/humble/setup.bash
cd ~/dev_ws/isaac_sim/smart_farm_spot/mockup

# 유방 추적 시연
python3 mock_gimbal_controller.py --target udder
```

출력 예시:
```
[udder] 중심=(645,358) 오차=(+5,-2) |e|=5px 보정=(sh0=-0.0040,sh1=+0.0016) 🎯 LOCK-ON (12프레임) → 📸 촬영 가능!
```

### A-4. 영상 직접 보기

```bash
# RGB
ros2 run rqt_image_view rqt_image_view /spot/arm/rgb/image_raw

# 열화상 (빨간 유방 = 열점)
ros2 run rqt_image_view rqt_image_view /spot/arm/thermal/image_raw
```

### A-5. 열화상 후처리 (유방염 열점 검출)

```bash
cd ~/dev_ws/isaac_sim/smart_farm_spot/ros2_bridge
python3 thermal_processor.py

# 결과 보기 (주황 박스 = 유방염 열점)
ros2 run rqt_image_view rqt_image_view /spot/arm/thermal/colormap
```

> ✅ 여기까지 되면 **엣지 파이프라인 전체(센서→검출→짐벌→열화상)가 검증**된 것입니다.

---

## 4. 시나리오 B: 보행 학습

> 로봇이 거친 지형에서 넘어지지 않고 걷도록 RL 학습.

### B-1. config 연결

```bash
cd ~/dev_ws/isaac_sim/IsaacLab

# config/spot 폴더를 IsaacLab 태스크 경로에 심볼릭 링크
ln -s ~/dev_ws/isaac_sim/smart_farm_spot/config/spot \
    source/extensions/omni.isaac.lab_tasks/omni/isaac/lab_tasks/manager_based/locomotion/velocity/config/spot_arm

# 태스크 재등록
pip install -e source/extensions/omni.isaac.lab_tasks --break-system-packages
```

### B-2. 학습 실행

```bash
source /home/rokey/dev_ws/venv/isaaclab/bin/activate
cd ~/dev_ws/isaac_sim/IsaacLab

python scripts/reinforcement_learning/rsl_rl/train.py \
    --task Isaac-Velocity-Rough-Spot-Arm-v0 \
    --num_envs 4096 \
    --headless
```

### B-3. 학습 모니터링

```bash
tensorboard --logdir logs/rsl_rl/spot_arm_rough
# 브라우저: http://localhost:6006
```

### B-4. 학습 결과 시연

```bash
python scripts/reinforcement_learning/rsl_rl/play.py \
    --task Isaac-Velocity-Rough-Spot-Arm-Play-v0 \
    --num_envs 32 \
    --load_run spot_arm_rough \
    --checkpoint model_3000.pt
```

| Iteration | 예상 시간 (RTX 5080) | 확인 지표 |
|-----------|---------------------|-----------|
| ~200 | ~40분 | 기립 (안 넘어짐) |
| ~500 | ~1.5시간 | 직진 보행 |
| ~1500 | ~5시간 | 거친 지형 통과 |
| ~3000 | ~12시간 | 안정 보행 |

---

## 5. 시나리오 C: 실제 소 에셋 연동

> 김현진의 소 에셋(USD)이 준비되면 가짜 소를 대체합니다.

### C-1. 통합 씬 구성

```
1. Isaac Sim에서 축사 씬 USD 열기 (소 + 여물통 + 통로 포함)
2. 로봇 추가: assets/spot_with_arm.usd 를 씬에 드래그 또는 reference
3. 소 객체에 시맨틱 라벨 부여 (아래)
```

### C-2. 실제 소에 시맨틱 라벨 부여

```python
# Script Editor
import sys
sys.path.insert(0, "/home/rokey/dev_ws/isaac_sim/smart_farm_spot/ros2_bridge")
from setup_semantics import tag_semantics, auto_tag_by_keyword, list_semantics

# 방법 1: 자동 (소 메쉬 이름에 udder/leg 키워드가 있으면)
auto_tag_by_keyword("/World")

# 방법 2: 수동 (정확한 prim 경로 지정)
tag_semantics("/World/Cow/Udder",     "udder")
tag_semantics("/World/Cow/RearLeg_L", "leg")
tag_semantics("/World/Cow/RearLeg_R", "leg")

list_semantics()   # 확인
```

### C-3. 브릿지 실행

```python
import isaac_ros2_bridge as bridge
bridge.setup_full_bridge(auto_semantics=False)  # 이미 수동 라벨링했으면 False
```

이후 시나리오 A의 확인 절차(A-2 ~ A-5)와 동일합니다.

### C-4. 헤드리스 통합 실행 (선택)

```bash
~/isaacsim/python.sh \
    ~/dev_ws/isaac_sim/smart_farm_spot/ros2_bridge/run_standalone.py \
    --usd /path/to/farm_scene.usd
```

---

## 6. 전체 체크리스트

### 환경
- [ ] Isaac Sim 5.1 실행 가능
- [ ] Isaac Lab venv 활성화
- [ ] ROS 2 Humble + vision_msgs 설치
- [ ] ROS_DOMAIN_ID 통일

### 목업 테스트 (시나리오 A)
- [ ] spot_with_arm.usd 정상 로드 (베이스 안 누움)
- [ ] mock_scene로 가짜 소 생성
- [ ] 브릿지 실행 후 Play
- [ ] mock_subscribers에서 5개 토픽 🟢
- [ ] mock_gimbal_controller에서 LOCK-ON 출력
- [ ] thermal_processor에서 열점 박스

### 학습 (시나리오 B)
- [ ] config/spot 심볼릭 링크
- [ ] 태스크 등록 성공
- [ ] train.py 실행 (4096 env)
- [ ] tensorboard 보상 상승 확인

### 실제 연동 (시나리오 C)
- [ ] 소 에셋 씬 로드
- [ ] 소에 시맨틱 라벨 부여
- [ ] /spot/arm/detections에 실제 소 박스

---

## 빠른 명령어 모음

```bash
# 토픽 목록
ros2 topic list | grep spot

# 객체 인식 결과
ros2 topic echo /spot/arm/detections

# 수신율 확인
ros2 topic hz /spot/arm/rgb/image_raw
ros2 topic hz /spot/imu/data

# 전체 상태 대시보드
python3 mockup/mock_subscribers.py

# 짐벌 시연
python3 mockup/mock_gimbal_controller.py --target udder
```

---

## 문제 발생 시

| 증상 | 해결 |
|------|------|
| 로봇이 뒤로 누움 | `spot_with_arm.usd` 최신본 사용 확인 (arm0_sh1=-149) |
| 토픽 안 보임 | Play(▶) 눌렀는지, ROS_DOMAIN_ID 일치 확인 |
| detections 0개 | 소에 시맨틱 라벨 부여 (`tag_semantics`) |
| 노드 타입 못 찾음 | `isaac_ros2_bridge.py`의 `USE_LEGACY=True` |
| 시뮬 느림 | 카메라 해상도 ↓, 학습 중엔 브릿지 OFF |

자세한 내용은 `docs/ROS2_BRIDGE_GUIDE.md` 참고.
