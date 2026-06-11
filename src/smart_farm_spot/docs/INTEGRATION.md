# 🔗 통합 내역 — spot_nav2 → smart_farm_spot

> 기존 두 패키지(`smart_farm_spot` 인지/RL + `spot_nav2` 주행)를
> **단일 ROS 2 패키지 `smart_farm_spot` v2.0** 으로 통합한 기록.

---

## 1. 무엇이 바뀌었나

### Isaac Sim 브릿지 병합 (핵심)
분리돼 있던 두 브릿지를 **하나의 스크립트**로 합쳤습니다.

| 기존 | 통합 후 |
|------|---------|
| `smart_farm_spot/ros2_bridge/isaac_ros2_bridge.py` (센서) | `isaac/isaac_sim_bridge.py` 의 `setup_sensor_bridge()` |
| `smart_farm_spot/ros2_bridge/run_standalone.py` (센서 러너) | `isaac/isaac_sim_bridge.py` 의 `main()` |
| `spot_nav2/isaac/isaac_nav_bridge.py` (odom/cmd_vel) | `isaac/isaac_sim_bridge.py` 의 `IsaacNavBridge` |

→ 기존엔 `isaac_nav_bridge.py --with-sensors` 가 다른 폴더의 `isaac_ros2_bridge` 를
  import 하려다 실패했음. 한 파일로 합쳐 **Isaac Sim 한 번 실행**으로 센서 + Nav 동시 동작.

**TF 소유권 정리:** 센서 OmniGraph의 TF 발행(`SENSOR_GRAPH_TF`)을 기본 `False` 로
끄고, TF(odom→base_link 동적 + base_link→센서 static)는 Nav 브릿지가 전담 →
이중 발행/프레임 충돌 방지.

### ROS 2 패키지 통합
| 기존 (spot_nav2) | 통합 후 (smart_farm_spot) |
|------------------|---------------------------|
| `spot_nav2/spot_nav2/waypoint_patrol.py` | `smart_farm_spot/waypoint_patrol.py` |
| (열화상 후처리 스크립트) | **비전 준비중 → 통합 제외**. 원본 `ros2_bridge/thermal_processor.py` 보관 |
| `config/nav2_params.yaml`, `waypoints.yaml` | `config/` 로 이동 |
| `maps/` | `maps/` 로 이동 |
| `launch/spot_nav2.launch.py` | `launch/spot_nav2.launch.py` (패키지명 갱신) |
| `launch/spot_slam.launch.py` | `launch/spot_slam.launch.py` |
| — | `launch/bringup.launch.py` ★신규 통합 런치★ |
| `ros2_bridge/setup_semantics.py` | `isaac/setup_semantics.py` |

### 엔트리포인트
```
ros2 run smart_farm_spot waypoint_patrol
# thermal_processor 는 비전 준비중 → 미등록 (완료 시 추가 예정)
```

### ⏸ 열화상 비전 (준비중 — 제외)
열화상 카메라 비전이 아직 준비중이라 통합에서 제외했습니다. 완료 시 재활성화:
1. `isaac/isaac_sim_bridge.py` 의 `ENABLE_THERMAL = True`
2. `ros2_bridge/thermal_processor.py` → `smart_farm_spot/thermal_processor.py` 로 이동
3. `setup.py` 에 `thermal_processor` 엔트리포인트 등록
4. `package.xml` 에 `cv_bridge` / `python3-opencv` 의존성 추가
5. `launch/bringup.launch.py` 에 thermal 노드/인자 복원
→ 토픽 `/spot/arm/thermal/image_raw`, `/spot/arm/thermal/colormap`

---

## 2. 명령어 마이그레이션

| 기존 | 통합 후 |
|------|---------|
| `~/isaacsim/python.sh .../spot_nav2/share/.../isaac_nav_bridge.py --with-sensors` | `~/isaacsim/python.sh $(ros2 pkg prefix smart_farm_spot)/share/smart_farm_spot/isaac/isaac_sim_bridge.py` |
| `ros2 launch spot_nav2 spot_nav2.launch.py` | `ros2 launch smart_farm_spot spot_nav2.launch.py` |
| `ros2 run spot_nav2 waypoint_patrol` | `ros2 run smart_farm_spot waypoint_patrol` |
| `colcon build --packages-select spot_nav2` | `colcon build --packages-select smart_farm_spot` |

---

## 3. 토픽

활성: `/clock` · `/spot/arm/rgb/image_raw` · `/spot/arm/detections` ·
`/spot/scan` · `/spot/imu/data` · `/odom` · `/cmd_vel` · `/tf`

제외(비전 준비중): `/spot/arm/thermal/image_raw` · `/spot/arm/thermal/colormap`

---

## 4. 기존 폴더 정리 (수동)

통합 검증 후 아래는 더 이상 필요 없습니다(원하면 삭제):

```
isaac_sim/spot_nav2/                 # → smart_farm_spot 로 흡수됨
isaac_sim/spot_nav2.zip
smart_farm_spot/ros2_bridge/         # isaac_ros2_bridge / run_standalone / setup_semantics
                                     #   → isaac/ 로 병합됨 (thermal_processor는 모듈로 이동)
```

> ⚠️ 이 워크스페이스는 git 관리가 아니므로, 삭제 전 동작 확인을 권장합니다.
> 통합본은 기존 파일을 **복사**해 만들었으므로 원본은 그대로 남아 있습니다.
