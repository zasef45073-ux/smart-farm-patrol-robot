# 할 일 (TODO) — 스마트 축사 자율순찰

_우선순위: 🔴발표 전(오늘) · 🟡GPU/Isaac 환경 · 🟢지금 가능(코드) · ⚪Phase 2(범위 밖)_

---

## 🔴 발표 전 (오늘 · 데모 성공 직결)

- [x] **대시보드 사전 1회 실행 확인** ✅ — uvicorn 기동·로그인 200·detect_mastitis 7/7·
      COW_6_10 등록·감지 end-to-end 검증 완료
- [ ] **데모 시나리오 = 대시보드 라이브 + Isaac은 녹화/보조** 로 구성 (Isaac 미검증 리스크 회피)
- [ ] Isaac 라이브 할 거면 **발표 전 반드시 한 번 띄워보기** (안 띄우면 ~30%, 띄워서 되면 그 장면 재현 ~90%)
- [x] `environment_final.usd` 를 `src/smart_farm_spot/assets/scene/` 에 배치 ✅ — git 커밋·push 완료, 전 isaac 스크립트 참조 통일

---

## 🟡 GPU/Isaac 환경에서 (실행·튜닝)

- [ ] **정책↔env 차원 일치 확인** — flat 정책 vs `Isaac-Velocity-Rough-SpotArm` 태스크 (안 맞으면 부팅 실패)
- [ ] Isaac 통합 실행: 순찰→검출→후방접근→검사 한 사이클
- [ ] **검사 자세 각도 튜닝** — `SF_INSP_SH1/EL0/WR0` (유방이 손 카메라에 들어오게)
- [ ] **앉기 자세 튜닝** — `SF_SIT_HY/KN` (몸 낮추되 z>0.30 유지)
- [ ] **앉기 leg-control 검증** — `env.step` 정책이 sit 을 거스르는지 / 정지구간 정책 leg-action 억제 필요 여부
- [x] keepout 마스크 생성 ✅ — `make_keepout_mask.py` 실행(2461×2461) + bringup/run_scenario 배선 완료
      ```bash
      python3 tools/make_keepout_mask.py
      ros2 launch smart_farm_spot bringup.launch.py keepout:=true   # 또는 SF_KEEPOUT=1 run_scenario.sh
      ```
      (남은 검증: Nav2 동반 실제 회피 동작 — Isaac 실행 필요)
- [ ] **4방향 카메라 + 헤드리스 1사이클** (신규) — 전/후/좌/우 카메라 + 순찰→검출→후방접근→검사
      완주를 GUI 없이(headless) 1회 자동 실행. ⚠️ Isaac/GPU 필요 — 작성 후 그 환경에서 검증.

---

## 🟢 지금 가능 (코드 마무리 · 데모 안정성↑)

- [x] **팔 무게 트릭 적용** ✅ — `arm_mass.py` 모듈(`apply_light_arm`) 병합, `scenario.py` 연동
- [x] **launch 배선** ✅ — `bringup.launch.py` 에 `twist_mux`/`dashboard`/`keepout` 인자(기본 off, 하위호환),
      `run_scenario.sh` 에 `SF_KEEPOUT`/`SF_DASHBOARD` 추가. colcon build·launch 파싱 검증 완료
- [x] **`/cmd_vel` 직접 발행 정리** ✅ — `cow_tail_seek` → `tail_search/cmd_vel`. `check_twist_mux.py` ✅(직접발행 0)
- [x] **카메라 토픽 연결** ✅ — 대시보드 `CAMERA_TOPIC=/spot_cam/rgb` 설정(.env.example/.env)
- [ ] **유방 Emission 머티리얼** — 소 USD 에 발광(40~42°C 모사) 추가 → hotspot 검출 동작
- [ ] **scenario 완주 연결** — `scenario_nav` 에 트래커(전 소 순회) 연동
- [ ] (선택) `cow_visual_servo` 재커밋 (검증됨, "검증만" 으로 미커밋)

---

## ⚪ Phase 2 (토이 범위 밖 · 상용화)

- [ ] 클라우드 Vision AI (PyTorch 파행/외상 정밀 판독)
- [ ] 파행 검출(뒷다리 포즈 기반)
- [ ] WebRTC 고화질 스트리밍 / PostgreSQL
- [ ] CI/CD·OTA (GitHub Actions), TLS/OAuth2/JWT, 오프라인 버퍼
- [ ] 질병 가상 에셋(파행 부종 메쉬)

---

## ✅ 이미 완료 (검증됨)

- 웹 대시보드(FastAPI+MQTT+SQLite+WS+SSE) — 전 엔드포인트 실행 검증, detect_mastitis 7/7
- 로봇 로직 — YOLO 트래킹·검사 시퀀스·twist_mux·keepout·브리지 등 **단위 테스트 47개 통과**
- SLAM static_layer + Keepout filter (투명벽 회피), 환경/맵 교체, 다중 소 배치
