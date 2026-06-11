# 시스템 아키텍처 — 스마트 축사 자율순찰 로봇 (구현 기준)

> 본 문서는 **실제 구현된 시스템** 아키텍처입니다. (설계 스펙 `System_Architecture_Detailed_Spec.md` 대비
> 일부 항목은 구현이 다름 — §8 차이표 참고.)

## 1. 개요
4족 로봇(Spot)이 Isaac Sim 축사를 **자율 순찰**하며 소를 **검출·검사**하고, 결과를 **웹 대시보드**로
관제하는 엣지–관제 통합 시스템. 엣지(로봇)=경량 검출, 관제(서버)=수집·표시·알림.

---

## 2. 시스템 통합 아키텍처 (Flowchart)

```mermaid
graph TD
    subgraph Edge["엣지 로봇 (Isaac Sim + ROS2 Humble)"]
        Sensors["RGB 카메라 / RTX LiDAR"] --> Bridge["Isaac 브릿지<br/>(scenario.py)"]
        Bridge --> YOLO["YOLO 검출 + 트래킹<br/>(cow_tracker)"]
        Bridge --> Nav2["Nav2 (경로·장애물회피)"]
        Nav2 --> RL["RL 보행정책<br/>(cmd_vel→12관절)"]
        YOLO --> Inspect["검사 시퀀스<br/>(정지→앉기→촬영)"]
        Inspect --> Capture["RGB-D 저장 + 유방염 hotspot"]
        TwistMux["twist_mux<br/>(우선순위·비상정지)"] --> RL
    end

    subgraph Net["농장 로컬망"]
        MQTT["MQTT (명령/상태)"]
        HTTP["HTTP/WS (영상·감지)"]
    end

    subgraph Cloud["관제 서버 (FastAPI)"]
        API["라우트: detection/camera/robot/edr"] --> DB[("SQLite<br/>detections·cattle·course·barn")]
        API --> SSE["SSE 실시간 알림"]
        API --> Dash["웹 대시보드<br/>(소상태·카메라·순찰·EDR)"]
    end

    Edge -->|상태/명령| MQTT
    Edge -->|영상·감지| HTTP
    MQTT <--> Cloud
    HTTP --> Cloud
    Cloud -->|/patrol/command| Bridge2["dashboard_bridge"]
    Bridge2 -->|e_stop / patrol| Edge
```

---

## 3. 엣지 제어 상태머신 (State Diagram)

```mermaid
stateDiagram-v2
    [*] --> PATROL : 순찰 시작
    state PATROL {
        [*] --> WaypointNav
        WaypointNav --> ObjectScan : 이동 중 YOLO 스캔
        ObjectScan --> WaypointNav
    }
    PATROL --> SEEK : 소 검출(트래커 확정)
    state SEEK {
        [*] --> ApproachRear : 후방(꼬리 1.5m) 접근
    }
    SEEK --> INSPECT : 후방 도착
    state INSPECT {
        [*] --> Stop : 정지(cmd_vel=0)
        Stop --> Sit : 앉기(다리 하강)
        Sit --> ArmDeploy : 팔 검사자세
        ArmDeploy --> Capture : RGB-D 촬영 + hotspot
        Capture --> Rise : 복귀
    }
    INSPECT --> PATROL : 검사완료 → 다음 소
    PATROL --> SAFETY : 비상정지(e_stop)
    SEEK --> SAFETY
    INSPECT --> SAFETY
    state SAFETY {
        [*] --> Lock : twist_mux 최상위 차단
    }
    SAFETY --> PATROL : 해제 후 순찰
    note right of INSPECT : 전 소 순회 완주(all_inspected)
    note left of SAFETY : 동물 충돌 방지 최우선
```

---

## 4. 검출·검사 시퀀스 (Sequence Diagram)

```mermaid
sequenceDiagram
    participant Cam as 손 RGB-D 카메라
    participant Robot as 엣지(scenario.py)
    participant Nav as Nav2 + RL
    participant Server as 대시보드(FastAPI)
    participant User as 농장주(브라우저)

    Robot->>Nav: 웨이포인트 순찰
    Cam->>Robot: 프레임
    Robot->>Robot: YOLO 검출 + 트래킹(track_id)
    Robot->>Nav: 소 후방(꼬리 1.5m) NavigateToPose
    Nav-->>Robot: 후방 도착(SUCCEEDED)
    Robot->>Robot: 정지→앉기→팔 검사자세
    Cam->>Robot: 고해상 RGB-D
    Robot->>Robot: detect_mastitis(빨강비율) / hotspot
    Robot->>Server: POST /api/save (감지) + /api/camera (영상)
    Server->>Server: DB 저장 + 소 상태 갱신
    Server->>User: SSE 실시간 알림(위험 팝업)
    Robot->>Robot: mark_inspected → 다음 소
```

---

## 5. 컴포넌트 구조 (Component Diagram)

```mermaid
graph TB
    subgraph EdgeWS["Edge: ROS2 워크스페이스 (smart_farm_spot)"]
        scenario["scenario.py<br/>Isaac 브릿지 + 검출 + 검사"]
        tracker["cow_tracker.py<br/>트랙 상태관리"]
        inspect["inspect_sequence.py<br/>앉기·검사 상태머신"]
        armposes["arm_poses.py<br/>팔/앉기 자세"]
        nav["Nav2 (nav2_params_slam)<br/>static+obstacle+keepout"]
        mux["twist_mux<br/>cmd_vel 중재"]
        dbridge["dashboard_bridge.py<br/>명령/상태"]
        scenario --> tracker
        scenario --> inspect --> armposes
        nav --> mux --> scenario
        dbridge --> mux
    end
    subgraph CloudSvc["Cloud: FastAPI 서비스"]
        routes["routes: detection/camera/robot/edr"]
        camstream["camera_stream.py<br/>YOLO 빨강비율"]
        mqttw["mqtt_client.py"]
        routes --> dbf[("SQLite")]
    end
    scenario -.->|영상| camstream --> routes
    dbridge <-.->|/patrol/command·/robot/status| routes
```

---

## 6. 레포 디렉터리 구조 (실제)

```text
D3_isaac_project/
├── src/smart_farm_spot/            # [Edge] ROS2 패키지
│   ├── smart_farm_spot/            #   ros2 run 노드(waypoint_patrol)
│   ├── isaac/                      #   Isaac 브릿지(scenario.py, nav_*bridge, scene_setup)
│   ├── cow_tracker.py · cow_tail_seek.py · dashboard_bridge.py
│   ├── arm_poses.py · inspect_sequence.py · inspection_capture.py
│   ├── config/                     #   nav2_params*·twist_mux·keepout_filter
│   ├── launch/                     #   spot_nav2·spot_slam·keepout·bringup
│   ├── maps/ · assets/             #   맵(점유격자) · USD/정책/YOLO
│   ├── tools/                      #   check_warmstart·check_twist_mux·make_keepout_mask
│   └── test/                       #   단위 테스트 47개
├── dashboard/                      # [Cloud] FastAPI 관제
│   ├── server.py · database.py · routes/ · templates/
│   ├── camera_stream.py · mqtt_client.py · ros2_bridge.py · detect_mastitis.py
│   └── docker-compose.yml · Dockerfile
├── TODO.md · TEST_COVERAGE_ANALYSIS.md
```

---

## 7. 통신 인터페이스 (실제 스키마)

| 채널 | 토픽/엔드포인트 | 페이로드 |
|------|-----------------|----------|
| ROS2 | `/cmd_vel` (Twist) | twist_mux 출력 → 보행 |
| ROS2 | `/patrol/command` (String) | `{command:start\|estop, course}` |
| ROS2 | `/robot/status` (String) | `{status:patrolling\|idle\|estop, course}` |
| ROS2 | `e_stop` (Bool) | twist_mux 비상정지 lock |
| HTTP | `POST /api/save` | `{cow_id, disease, severity, timestamp}` |
| HTTP | `POST /api/camera` | `{image: base64 jpeg}` → WS 송출 |
| HTTP | `GET /api/summary` | `{danger, normal}` |
| SSE  | `GET /api/stream` | 감지 이벤트 실시간 푸시 |
| MQTT | `cattle/detection` | 감지 → `/api/save` 중계 |

---

## 8. 맵 전략 (SLAM + Prior-map + Keepout)

```mermaid
graph LR
    Lidar["RTX LiDAR /scan"] --> SLAM["slam_toolbox<br/>(라이브 매핑)"]
    Barn["축사 USD 콜라이더<br/>(투명 가상벽 포함)"] --> Server["barn_map_server<br/>(사전맵 주입)"]
    Server --> Mask["make_keepout_mask<br/>→ keepout 마스크"]
    SLAM --> Map["/map"]
    Server --> Map
    Map --> Static["static_layer"]
    Mask --> Keep["KeepoutFilter"]
    Static --> Costmap["Nav2 코스트맵"]
    Keep --> Costmap
    Costmap --> Plan["경로계획·실시간 회피<br/>(투명벽 포함)"]
```

---

## 8b. 설계 스펙 대비 구현 차이

| 항목 | 스펙(계획) | 구현(실제) |
|------|-----------|-----------|
| 질병 대상 | 파행/외상 (클라우드 Vision AI) | 유방염/외상 (빨강비율, edge) |
| 클라우드 | Flask + PostgreSQL | FastAPI + SQLite |
| 영상 | WebRTC | HTTP base64 + WebSocket |
| 정밀 판독 | PyTorch 클라우드 추론 | 미구현(Phase 2), edge 사전탐지만 |
| 인증/보안 | OAuth2/JWT/TLS | 세션 로그인 |

---

## 9. 핵심 설계 결정
- **엣지–관제 분리**: 로봇은 경량 검출, 서버는 수집·표시·알림.
- **이중 맵 전략**: 라이브 SLAM + Isaac 사전맵 주입(4족 진동 대비) + 투명벽 Keepout.
- **안전 최우선**: twist_mux 비상정지 최상위 + 충돌회피.
- **무게 트릭 보행**: 팔 무게≈0 → 무팔 RL 정책 재사용(재학습 0) — 검사 중 팔 사용.
- **학습 불요 파이프라인**: YOLO 트래킹·검사·검출 전부 기존 가중치/룰 기반.
