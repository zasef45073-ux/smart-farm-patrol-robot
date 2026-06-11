# Copyright (c) 2024, Smart Farm Patrol Project - 꼬마 로봇 두리
# SPDX-License-Identifier: Apache-2.0
"""
inspect_sequence.py
===================
**후방 도착 → 앉기 → 유방 검사 → 복귀** 상태머신 — 순수 로직(Isaac/torch 불요).

step(프레임) 기반으로 각 단계 진행도를 산출. scenario.py 가 이 진행도로
다리(앉기)·팔(검사자세) position target 을 lerp 하고, 촬영 시점을 잡는다.

단계:
  IDLE → SIT(다리 하강) → DEPLOY(팔 검사자세) → HOLD(촬영) →
  RETRACT(팔 복귀) → RISE(일어서기) → DONE

반환 leg_t/arm_t 는 0..1 진행도:
  leg_t=1 → 완전히 앉음,  arm_t=1 → 검사자세 완성.
정지(cmd_zero)는 SIT~RISE 내내 True(검사 중 주행 금지).
"""
from collections import namedtuple

InspectState = namedtuple(
    "InspectState", ["phase", "active", "cmd_zero", "leg_t", "arm_t", "capture", "done"])


class InspectSequence:
    PHASES = ("IDLE", "SIT", "DEPLOY", "HOLD", "RETRACT", "RISE", "DONE")

    def __init__(self, sit_steps=120, deploy_steps=120, hold_steps=120,
                 retract_steps=120, rise_steps=120):
        """각 단계 길이(프레임). 60Hz 기준 120=~2s."""
        self.sit = max(1, sit_steps)
        self.deploy = max(1, deploy_steps)
        self.hold = max(1, hold_steps)
        self.retract = max(1, retract_steps)
        self.rise = max(1, rise_steps)
        self._start = None        # 시퀀스 시작 step
        self._captured = False

    # ── 제어 ──
    def start(self, step):
        """후방 도착 시 호출 — 검사 시퀀스 시작."""
        self._start = step
        self._captured = False

    def reset(self):
        self._start = None
        self._captured = False

    def is_active(self):
        return self._start is not None

    # ── 매 프레임 ──
    def update(self, step):
        if self._start is None:
            return InspectState("IDLE", False, False, 0.0, 0.0, False, False)
        e = step - self._start          # 시퀀스 내 경과
        t1 = self.sit
        t2 = t1 + self.deploy
        t3 = t2 + self.hold
        t4 = t3 + self.retract
        t5 = t4 + self.rise

        capture = False
        if e < t1:                      # SIT: 다리 하강
            phase, leg_t, arm_t = "SIT", e / self.sit, 0.0
        elif e < t2:                    # DEPLOY: 팔 검사자세
            phase, leg_t, arm_t = "DEPLOY", 1.0, (e - t1) / self.deploy
        elif e < t3:                    # HOLD: 촬영
            phase, leg_t, arm_t = "HOLD", 1.0, 1.0
            if not self._captured:      # HOLD 진입 첫 프레임에 1회 촬영
                capture = True
                self._captured = True
        elif e < t4:                    # RETRACT: 팔 복귀
            phase, leg_t, arm_t = "RETRACT", 1.0, 1.0 - (e - t3) / self.retract
        elif e < t5:                    # RISE: 일어서기
            phase, leg_t, arm_t = "RISE", 1.0 - (e - t4) / self.rise, 0.0
        else:                           # DONE
            return InspectState("DONE", False, False, 0.0, 0.0, False, True)

        leg_t = 0.0 if leg_t < 0 else (1.0 if leg_t > 1 else leg_t)
        arm_t = 0.0 if arm_t < 0 else (1.0 if arm_t > 1 else arm_t)
        return InspectState(phase, True, True, leg_t, arm_t, capture, False)
