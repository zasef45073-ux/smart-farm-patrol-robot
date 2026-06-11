# 📡 Spot ROS 2 브릿지 사용 가이드

> Isaac Sim 5.1 + ROS 2 Humble  
> 카메라 · LiDAR · IMU · **Isaac Sim 내부 객체 인식**

---

## 핵심 설계: 객체 인식을 Isaac Sim 안에서 처리

### 왜 이렇게 하나 (최적화)

기존 계획은 엣지 로봇에서 YOLO 같은 객체 탐지 모델을 돌리는 것이었습니다.  
하지만 시뮬레이션(SITL)에서는 **Isaac Sim이 제공하는 "정답 Bounding Box"** 를 쓰는 게 훨씬 효율적입니다.

| 구분 | 기존 (엣지 YOLO) | 변경 (Isaac Sim 내부) |
|------|------------------|----------------------|
| 추론 비용 | GPU 점유 (모델 추론) | **0** (렌더 파이프라인 부산물) |
| 정확도 | 학습 품질 의존 | **100%** (정답 데이터) |
| 구현 시간 | 모델 학습 필요 | 라벨만 부여 |
| 5일 SITL 적합성 | 낮음 | **높음** |

소의 유방·다리 메쉬에 **시맨틱 라벨**만 붙이면, 카메라가 바라볼 때  
`bbox_2d_tight` 어노테이터가 자동으로 Bounding Box를 `/spot/arm/detections`로 발행합니다.

> 실제 질병 판독(유방염/파행)은 여전히 클라우드에서 수행합니다.  
> 엣지의 "객체가 어디 있는가" 탐지만 Isaac Sim 정답으로 대체하는 것입니다.

---

## 발행 토픽

| 토픽 | 타입 | 주기 | 용도 |
|------|------|------|------|
| `/clock` | `rosgraph_msgs/Clock` | sim | 시간 동기화 |
| `/spot/arm/rgb/image_raw` | `sensor_msgs/Image` | 30Hz | RGB 영상 (파행/촬영) |
| `/spot/arm/detections` | `vision_msgs/Detection2DArray` | 30Hz | **★객체 인식★** |
| `/spot/arm/thermal/image_raw` | `sensor_msgs/Image` | 15Hz | 열화상 (유방염) |
| `/spot/scan` | `sensor_msgs/LaserScan` | 10Hz | LiDAR (Nav2) |
| `/spot/imu/data` | `sensor_msgs/Imu` | 200Hz | IMU (보행/짐벌) |
| `/tf` | `tf2_msgs/TFMessage` | sim | 좌표 변환 |

---

## 실행 방법

### 방법 A — Isaac Sim GUI (시각 확인용)

```
1. Isaac Sim 실행
2. assets/spot_with_arm.usd 열기 (또는 소+축사 통합 씬)
3. Window > Extensions → "isaacsim.ros2.bridge" 활성화 확인
4. Window > Script Editor 열기
5. ros2_bridge/isaac_ros2_bridge.py 내용 전체 붙여넣기 → 실행(▶)
6. 상단 Play(▶) 버튼 클릭 → 토픽 발행 시작
```

### 방법 B — Standalone 헤드리스 (학습/연동용)

```bash
# 로봇 단독 (기본 지면)
~/isaacsim/python.sh ros2_bridge/run_standalone.py

# 소+축사 통합 씬 로드
~/isaacsim/python.sh ros2_bridge/run_standalone.py --usd /path/to/farm_scene.usd

# GUI 같이 보기
~/isaacsim/python.sh ros2_bridge/run_standalone.py --gui
```

---

## 객체 인식 라벨 설정 (필수)

`/spot/arm/detections`에 박스가 나오려면 소 객체에 시맨틱 라벨이 있어야 합니다.

### 자동 라벨링 (이름 키워드 기반)

```python
# Script Editor 에서
from setup_semantics import auto_tag_by_keyword
auto_tag_by_keyword("/World")
# prim 이름에 udder/leg/cow 등이 포함되면 자동 라벨링
```

### 수동 라벨링 (정확한 경로 지정)

```python
from setup_semantics import tag_semantics, list_semantics

tag_semantics("/World/Cow_01/Udder",      "udder")   # 유방
tag_semantics("/World/Cow_01/RearLeg_L",  "leg")     # 왼뒷다리
tag_semantics("/World/Cow_01/RearLeg_R",  "leg")     # 오른뒷다리
tag_semantics("/World/Cow_01",            "cow")     # 소 전체

list_semantics()   # 부여된 라벨 확인
```

라벨 클래스는 `udder`(유방), `leg`(다리), `cow`(소 전체) 세 가지입니다.

---

## 확인 명령어

```bash
# 토픽 목록
ros2 topic list | grep spot

# 객체 인식 결과 (Bounding Box)
ros2 topic echo /spot/arm/detections

# RGB 영상 보기
ros2 run rqt_image_view rqt_image_view /spot/arm/rgb/image_raw

# 열화상 영상 보기
ros2 run rqt_image_view rqt_image_view /spot/arm/thermal/image_raw

# LiDAR 스캔 (Nav2 확인)
ros2 topic echo /spot/scan --once

# IMU
ros2 topic hz /spot/imu/data
```

---

## Detection2DArray 메시지 구조

`/spot/arm/detections`로 발행되는 객체 인식 결과:

```
vision_msgs/Detection2DArray
├── header (frame_id: arm_rgb_frame)
└── detections[]
    ├── bbox
    │   ├── center.position.x  (박스 중심 x 픽셀)
    │   ├── center.position.y  (박스 중심 y 픽셀)
    │   ├── size_x             (박스 너비)
    │   └── size_y             (박스 높이)
    └── results[]
        └── hypothesis.class_id  ("udder" / "leg" / "cow")
```

RL 짐벌 제어기는 이 `center.position`을 카메라 화면 중앙(640, 360)과 비교해  
오차 $e_x, e_y$를 계산하고 팔 관절을 조정하여 Lock-on 합니다.

---

## 자주 발생하는 문제

### `/spot/arm/detections`에 박스가 안 나옴
→ 소 객체에 시맨틱 라벨이 없습니다. `tag_semantics()`로 라벨을 부여하세요.  
→ 카메라 시야에 라벨된 객체가 들어와야 박스가 출력됩니다.

### `ModuleNotFoundError: isaacsim.ros2.bridge`
→ ROS 2 Bridge 익스텐션이 비활성화 상태입니다.  
→ GUI: Window > Extensions에서 검색 후 활성화  
→ 코드: `enable_extension("isaacsim.ros2.bridge")` 호출 확인

### 노드 타입을 못 찾음 (`isaacsim.ros2.bridge.ROS2CameraHelper` 없음)
→ Isaac Sim 버전이 구버전(2023.x)일 수 있습니다.  
→ `isaac_ros2_bridge.py` 상단의 `USE_LEGACY = False`를 `True`로 변경하세요.  
   (네임스페이스가 `omni.isaac.*`로 전환됩니다.)

### 토픽이 안 보임 (ros2 topic list 비어있음)
→ Play(▶) 버튼을 눌렀는지 확인 (재생 중에만 발행).  
→ `ROS_DOMAIN_ID`가 Isaac Sim과 터미널에서 같은지 확인.
→ `source /opt/ros/humble/setup.bash` 했는지 확인.

### 영상 FPS가 낮음 / 시뮬이 느림
→ 카메라 2대 동시 렌더는 무겁습니다.  
→ `RGB_RESOLUTION`, `THERMAL_RESOLUTION`을 낮추세요.  
→ 학습 중에는 브릿지를 끄고, 시연 시에만 켜는 것을 권장합니다.

---

## 데이터 흐름

```
[Isaac Sim]
  RGB 카메라 ──┬─→ rgb 영상 ─────────→ /spot/arm/rgb/image_raw ──→ (클라우드 촬영본)
              └─→ bbox_2d_tight ────→ /spot/arm/detections ────→ RL 짐벌 제어기 (Lock-on)
                  (시맨틱 라벨 기반)

  Thermal 카메라 → rgb 영상 ─────────→ /spot/arm/thermal/image_raw ─→ thermal_processor.py
                                                                        └→ JET colormap + 열점 bbox

  LiDAR ───────→ laser_scan ────────→ /spot/scan ──→ Nav2 Collision Avoidance
  IMU ─────────→ imu ───────────────→ /spot/imu/data ─→ 보행/짐벌 제어
```
