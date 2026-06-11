# 맵 파일 (maps/)

Nav2 AMCL 위치 추정용 사전 맵을 둡니다.

```
maps/
├── barn_map.yaml    # 맵 메타데이터
└── barn_map.pgm     # 맵 이미지 (점유 격자)
```

## 생성 방법

### SLAM Toolbox 매핑
```bash
ros2 launch spot_nav2 spot_slam.launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard   # 수동 주행
ros2 run nav2_map_server map_saver_cli -f barn_map     # 저장
```

### Digital Twin 사전 맵 추출 (권장)
Isaac Sim 가상 환경에서 노이즈 없는 점유 격자를 직접 내보내 주입.

## barn_map.yaml 예시
```yaml
image: barn_map.pgm
mode: trinary
resolution: 0.05
origin: [-5.0, -5.0, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
```

## 맵 없이 테스트
```bash
ros2 launch spot_nav2 spot_nav2.launch.py use_amcl:=false
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom
```
