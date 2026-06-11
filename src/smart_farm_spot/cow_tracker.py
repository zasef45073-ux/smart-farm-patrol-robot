# Copyright (c) 2024, Smart Farm Patrol Project - 꼬마 로봇 두리
# SPDX-License-Identifier: Apache-2.0
"""
cow_tracker.py
==============
**YOLO 소 트랙 상태관리** — 순수 로직(ROS/Isaac/YOLO 불요, 학습 불요).

기존 `best.pt` + `model.track(persist=True)`(ByteTrack/BoT-SORT)가 주는
track_id 단위 검출을 받아:
  · 연속 N프레임 확인 후에만 "확정"(플리커/오검출 흡수)
  · 일정 시간 안 보이면 트랙 소멸(드롭아웃 처리)
  · 검사 대상 1마리를 **락(lock)** — 검사 끝날 때까지 다른 소로 안 튐
  · 검사 완료 track_id 기록 → **같은 소 반복 안 함 / 전 소 순회 완주**

per-frame detection = dict: {"id": int, "conf": float, "cx": float, "cy": float, ...}
"""


class CowTrackManager:
    """track_id 기반 소 검출 안정화 + 검사-완료 관리."""

    def __init__(self, min_hits=3, max_age=0.5, lock_until_inspected=True):
        """
        Args:
            min_hits:  확정에 필요한 누적 검출 프레임 수(플리커 흡수)
            max_age:   이 시간(s) 동안 미검출이면 트랙 소멸
            lock_until_inspected: 대상 소를 검사 완료/소멸까지 고정
        """
        self.min_hits = min_hits
        self.max_age = max_age
        self.lock = lock_until_inspected
        self._tracks = {}        # id -> {"hits", "last_seen", "obs"}
        self.inspected = set()   # 검사 완료한 track_id
        self.target_id = None    # 현재 검사 대상

    # ── 매 프레임 갱신 ──
    def update(self, detections, now):
        """이번 프레임 검출 반영 + 노화 처리 → 현재 대상 track_id 반환(None=없음)."""
        for d in detections:
            tid = d["id"]
            t = self._tracks.get(tid)
            if t is None:
                self._tracks[tid] = {"hits": 1, "last_seen": now, "obs": d}
            else:
                t["hits"] += 1
                t["last_seen"] = now
                t["obs"] = d
        # 노화: max_age 초과 미검출 트랙 제거
        for tid in list(self._tracks):
            if now - self._tracks[tid]["last_seen"] > self.max_age:
                del self._tracks[tid]
                if self.target_id == tid:
                    self.target_id = None
        return self._select()

    # ── 확정 트랙 ──
    def confirmed(self):
        """min_hits 이상 누적된 확정 track_id 목록."""
        return [tid for tid, t in self._tracks.items() if t["hits"] >= self.min_hits]

    def _select(self):
        # 락: 현재 대상이 아직 확정·미검사면 유지
        if self.lock and self.target_id is not None:
            t = self._tracks.get(self.target_id)
            if t and t["hits"] >= self.min_hits and self.target_id not in self.inspected:
                return self.target_id
            self.target_id = None
        # 확정 + 미검사 중 최고 conf 선택
        cands = [(t["obs"]["conf"], tid) for tid, t in self._tracks.items()
                 if t["hits"] >= self.min_hits and tid not in self.inspected]
        self.target_id = max(cands)[1] if cands else None
        return self.target_id

    def target_obs(self):
        """현재 대상의 마지막 관측 dict(없으면 None)."""
        t = self._tracks.get(self.target_id) if self.target_id is not None else None
        return t["obs"] if t else None

    # ── 검사 완료 ──
    def mark_inspected(self, track_id=None):
        """대상(또는 지정 track_id) 검사 완료 처리 → 다음 소로 넘어감."""
        tid = self.target_id if track_id is None else track_id
        if tid is not None:
            self.inspected.add(tid)
            if self.target_id == tid:
                self.target_id = None

    def all_inspected(self, expected_ids=None):
        """전 소 검사 완주 여부. expected_ids 미지정 시 지금까지 본 소 기준."""
        target = set(expected_ids) if expected_ids is not None \
            else (set(self._tracks) | self.inspected)
        return bool(target) and target.issubset(self.inspected)
