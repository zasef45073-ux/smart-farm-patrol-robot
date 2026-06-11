# 📦 에셋 명세서 (Asset Specification)

**프로젝트**: 꼬마 로봇 두리 — Spot + Arm 자율순찰 시뮬레이션
**문서 버전**: 1.0
**작성일**: 2026-06-09
**상태**: Final

본 문서는 Isaac Sim 시뮬레이션에 사용되는 모든 에셋(USD 로봇, 센서, 목업)의 정식 명세를 정의합니다.

---

## 1. USD 에셋 파일

| 파일 | 팔 | 참조 소스 | 상태 | 용도 |
|------|----|-----------|----|------|
| `assets/spot_with_arm.usd` | ✅ 있음 | NVIDIA `spot_with_arm.usd` (외부 subLayer) | 동작 검증 완료 | **메인** — 유방/다리 근접 촬영 |
| `assets/spot_only.usd` | ❌ 없음 | NVIDIA `spot.usd` (외부 subLayer) | 동작 검증 완료 | 보행 학습 단순화 버전 |
| `assets/build_sensors_usd.py` | — | — | 유틸리티 | 원본에서 센서 USD 재빌드 |

### 1.1 공통 구조 원칙

두 USD 모두 다음 원칙을 따릅니다.

- **외부 참조 구조**: NVIDIA 원본 USD를 `subLayers`로 참조하고, 그 위에 `over`로 센서·조인트 수정만 얹습니다. 원본 메쉬·물리를 직접 수정하지 않아 안정적입니다.
- **로드 안정성 확보**: 로컬 self-reference, `/Render` 뷰포트 블록, `/physicsScene` 중복 블록, 빈 `isaac:namespace` 속성을 모두 제거했습니다. 어느 환경에서든 cycle 경고 없이 로드됩니다.
- **defaultPrim**: `spot`

```
subLayers = [
    @https://omniverse-content-production.s3-us-west-2.amazonaws.com/
      Assets/Isaac/5.1/Isaac/Robots/BostonDynamics/spot/spot_with_arm.usd@
]
```

---

## 2. 탑재 센서 (spot_with_arm.usd)

| 센서 | prim 경로 | 규격 | 위치·자세 | ROS 2 토픽 | 메시지 타입 | 용도 |
|------|-----------|------|-----------|-----------|------------|------|
| RGB Camera | `/spot/arm0_link_fngr/RGBCamera` | 1280×720 @30FPS · 광각 ~82° | end-effector 앞쪽, 하향 -90° | `/spot/arm/rgb/image_raw` | `sensor_msgs/Image` | 파행 탐지 + Lock-on 촬영 |
| Thermal Camera | `/spot/arm0_link_fngr/ThermalCamera` | 640×512 @15FPS · 협각 ~52° | end-effector 옆쪽, 하향 -90° | `/spot/arm/thermal/image_raw` | `sensor_msgs/Image` | 유방염 열점(40~42°C) |
| LiDAR 360° | `/spot/base/Lidar360/lidar_sensor` | VLP-16 · 16ch · 0.2° · 30m · 10Hz | base 상단 +0.3m | `/spot/scan` | `sensor_msgs/LaserScan` | Nav2 충돌 회피 + SLAM |
| IMU | `/spot/base/IMU` | 200Hz (RPY + 가속도) | base 무게중심 | `/spot/imu/data` | `sensor_msgs/Imu` | 보행/짐벌 흔들림 보정 |

### 2.1 카메라 상세 파라미터

| 속성 | RGBCamera | ThermalCamera |
|------|-----------|---------------|
| `focalLength` | 12 | 13 |
| `horizontalAperture` | 20.955 | 12.7 |
| `clippingRange` | (0.05, 10) | (0.05, 8) |
| `focusDistance` | 0.8 | 0.6 |
| `translate` | (0.05, 0, -0.05) | (-0.05, 0, -0.05) |
| `rotateXYZ` | (-90, 0, 0) | (-90, 0, 0) |

### 2.2 LiDAR 상세 파라미터 (IsaacLidarAPI)

| 속성 | 값 | 비고 |
|------|----|----|
| `numBeams` | 16 | VLP-16 채널 |
| `horizontalResolution` | 0.2° | 회전당 1800포인트 |
| `minVerticalAngle` / `maxVerticalAngle` | -15° / +15° | 수직 FOV |
| `minHorizontalAngle` / `maxHorizontalAngle` | 0° / 360° | 풀 스캔 |
| `minRange` / `maxRange` | 0.2m / 30m | 거리 범위 |
| `rotationRate` | 10Hz | Nav2 기본값 |
| `drawPoints` | false | 학습 중 성능 확보 |

### 2.3 IMU 상세 파라미터

| 속성 | 값 |
|------|----|
| `sensorFrequency` | 200Hz |
| `accelNoiseDensity` | 0.01 |
| `gyroNoiseDensity` | 0.001 |

---

## 3. 센서 구성 (spot_only.usd)

팔이 없는 버전으로, 카메라를 `base` 전방에 직접 장착합니다.

| 센서 | prim 경로 | 위치 | 용도 |
|------|-----------|------|------|
| RGB Camera | `/spot/base/RGBCamera` | base 전방 (수평) | 전방 객체 인식 |
| LiDAR 360° | `/spot/base/Lidar360/lidar_sensor` | base 상단 +0.3m | Nav2 + SLAM |
| IMU | `/spot/base/IMU` | base 무게중심 | 보행 제어 |

---

## 4. 객체 인식 에셋 (Isaac Sim 내부)

별도 하드웨어 센서가 아니라, RGB 카메라 렌더에서 시맨틱 라벨 기반으로 정답 Bounding Box를 추출하는 방식입니다.

| 항목 | 내용 |
|------|------|
| 발행 토픽 | `/spot/arm/detections` |
| 메시지 타입 | `vision_msgs/Detection2DArray` |
| 어노테이터 | `bbox_2d_tight` |
| 입력 | RGB 카메라 렌더 프로덕트 |
| 필수 조건 | 대상 객체에 시맨틱 라벨 부여 |

### 4.1 시맨틱 라벨 클래스

| 클래스 | 대상 | 관찰 질병 |
|--------|------|-----------|
| `udder` | 소 유방 | 유방염 (Mastitis) |
| `leg` | 소 뒷다리 | 파행 (Lameness) |
| `cow` | 소 전체 | 접근/회피 기준 |

> **최적화 근거**: 엣지에서 YOLO 등 딥러닝 추론을 돌리는 대신 시뮬레이터의 정답 데이터를 사용합니다. 추론 비용 0, 정확도 100%로 5일 SITL 데모에 최적이며, 실제 질병 판독(유방염/파행)은 클라우드가 담당합니다.

---

## 5. 목업 소 에셋 (mockup/mock_scene.py)

실제 소 에셋(Day 1 제작) 준비 전, 파이프라인 검증용 도형 기반 가짜 소입니다.

| 구성요소 | 도형 | 위치 (소 기준) | 시맨틱 라벨 | 모사 대상 |
|----------|------|---------------|------------|-----------|
| 몸통(Body) | 가로 실린더 (r0.35, h1.2) | (0, 0, 0.9) | `cow` | 소 전체 |
| 유방(Udder) | 빨간 발광 구체 (r0.18) | (-0.45, 0, 0.55) | `udder` | 유방 + 열점(Emission ×3) |
| 정상 다리 | 얇은 박스 (0.08³) | (-0.4, +0.25, 0.4) | `leg` | 건강한 뒷다리 |
| 부은 다리 | 굵은 박스 (0.18³) | (-0.4, -0.25, 0.35) | `leg` | 파행 (관절 부종) |

- 배치 기본 위치: 로봇 전방 1.2m `(1.2, 0.0, 0.0)`
- `build_mock_farm()` 사용 시 소 3마리 + 여물통 미니 축사 생성

---

## 6. 조인트 수정 명세 (소환 안정화)

NVIDIA 원본 USD의 팔 조인트 `drive:angular:physics:targetPosition` 극단값이 물리 시작 시 베이스를 뒤집는 문제를 수정했습니다.

| 조인트 | 원본 | 수정 | stiffness / damping | 이유 |
|--------|------|------|---------------------|------|
| `arm0_el0` (elbow) | 170° | **0°** | 400 / 40 | 극단 꺾임 반작용 → 베이스 전복 |
| `arm0_sh1` (shoulder pitch) | -175° | **-149°** | 400 / 40 | CoM 후방 이탈 방지, 앞쪽 하향 접힘 |
| `arm0_sh0` (shoulder yaw) | — | 0° | 400 / 40 | 중립 |
| `arm0_el1` | — | 0° | 400 / 40 | 중립 |
| `arm0_wr0` / `arm0_wr1` | — | 0° | 200 / 20 | 손목 중립 |
| `arm0_f1x` (gripper) | — | 0° | 100 / 10 | 그리퍼 중립 |

다리 12관절은 IsaacLab 설정(`spot_arm.py`)의 `init_state.joint_pos`에서 안정 스탠딩 자세(`hy=0.7, kn=-1.4`)로 초기화됩니다.

---

## 7. 설계 의도 요약

### 센서 배치
카메라 2대를 `arm0_link_fngr`(팔 끝)에 장착한 이유는, 팔을 하강(Downward)시켰을 때 소 하부(유방·다리)를 정면 하향으로 촬영할 수 있기 때문입니다. 반면 LiDAR·IMU는 로봇 전체의 주변 인식·자세 추정 기준이므로 `base`에 고정합니다.

### 에셋 분담
`spot_only.usd`로 보행을 먼저 안정화하고, `spot_with_arm.usd`로 팔 포함 촬영 시나리오를 검증하는 단계적 접근이 가능합니다.

### 열화상 모사
Isaac Sim은 네이티브 열화상 센서를 미지원하므로, 유방 머티리얼에 강한 Emission을 부여하고 RGB 카메라로 촬영한 뒤 OpenCV JET colormap으로 후처리하여 열화상을 모사합니다 (`ros2_bridge/thermal_processor.py`).

---

## 8. 파일 인벤토리

```
smart_farm_spot/
├── assets/
│   ├── spot_with_arm.usd          # 메인 로봇 (팔 + 센서 4종)
│   ├── spot_only.usd              # 팔 없는 버전 (센서 3종)
│   └── build_sensors_usd.py       # USD 재빌드 유틸
├── config/spot/                   # Isaac Lab 학습 설정
│   ├── spot_arm.py                # ArticulationCfg (팔 버전)
│   ├── spot_only.py               # ArticulationCfg (팔 없는 버전)
│   ├── rough_arm_env_cfg.py       # 환경 설정
│   ├── __init__.py                # gym 등록
│   └── agents/rsl_rl_ppo_cfg.py   # PPO 하이퍼파라미터
├── ros2_bridge/
│   ├── isaac_ros2_bridge.py       # ROS 2 브릿지 (센서 → 토픽)
│   ├── setup_semantics.py         # 시맨틱 라벨링
│   ├── run_standalone.py          # 헤드리스 러너
│   └── thermal_processor.py       # 열화상 후처리
├── mockup/
│   ├── mock_scene.py              # 가짜 소 씬
│   ├── mock_subscribers.py        # 토픽 검증 대시보드
│   └── mock_gimbal_controller.py  # Lock-on 흐름 시연
├── sensors/
│   ├── spot_arm_sensor_cfg.py     # Isaac Lab 센서 CFG
│   └── setup_spot_sensors.py      # Script Editor 센서 셋업
└── docs/
    ├── README.md                  # 패키지 개요
    ├── RUN_GUIDE.md               # 실행 가이드
    ├── ROS2_BRIDGE_GUIDE.md       # 브릿지 상세
    └── ASSETS.md                  # 본 문서
```
