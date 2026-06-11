#!/usr/bin/env python3
# Copyright (c) 2024, Smart Farm Patrol Project - 꼬마 로봇 두리
# SPDX-License-Identifier: Apache-2.0
"""
check_twist_mux.py
==================
**twist_mux 우선순위 중재 검증** — /cmd_vel 다중 발행 충돌이 실제로 제거됐는지 확인.

두 가지를 검사한다:
  [A] config/twist_mux.yaml 자체가 건전한지
        · 각 input: topic/timeout/priority 존재
        · priority 전부 고유(중복이면 중재 모호)
        · input timeout > 0 (0 이면 만료 안 돼 stale 명령이 락)
        · 최상위 우선순위 lock(e_stop) 존재 → 비상정지 가능
  [B] 코드가 **/cmd_vel 로 직접 발행**하는 노드를 남겨두지 않았는지
        (twist_mux 도입 후엔 각 명령원이 input 토픽으로 발행해야 함.
         /cmd_vel 직접 발행은 mux 출력을 침범 → 충돌 부활)
        · 구독(create_subscription)은 정상(= mux 출력 소비자)이라 제외

실행:
  python3 tools/check_twist_mux.py
  python3 tools/check_twist_mux.py --config config/twist_mux.yaml --src .
"""
import argparse
import glob
import os
import re
import sys

import yaml


def check_config(path):
    print(f"[A] config 검사: {path}")
    ok = True
    with open(path) as f:
        data = yaml.safe_load(f)
    try:
        params = data["twist_mux"]["ros__parameters"]
    except (KeyError, TypeError):
        print("    ❌ twist_mux/ros__parameters 구조 아님"); return False
    topics = params.get("topics", {}) or {}
    locks = params.get("locks", {}) or {}
    if not topics:
        print("    ❌ topics 비어있음"); ok = False

    prios = {}
    for name, c in {**topics, **locks}.items():
        for key in ("topic", "timeout", "priority"):
            if key not in c:
                print(f"    ❌ '{name}': '{key}' 누락"); ok = False
        p = c.get("priority")
        if p in prios:
            print(f"    ❌ priority 중복: '{name}' 와 '{prios[p]}' 둘 다 {p}"); ok = False
        prios[p] = name

    for name, c in topics.items():
        if c.get("timeout", 0) <= 0:
            print(f"    ❌ input '{name}' timeout={c.get('timeout')} ≤0 "
                  f"(만료 안 돼 끊긴 명령이 락 → 정지 못 함)"); ok = False

    if not locks:
        print("    ⚠️ lock(e_stop) 없음 — 비상정지 우선차단 불가(권장: 최상위 lock 추가)")
    else:
        top = max(prios)
        if top not in [c["priority"] for c in locks.values()]:
            print(f"    ❌ 최상위 priority({top})가 lock 이 아님 — e_stop 이 최우선이어야 안전"); ok = False
        else:
            print(f"    ✅ e_stop lock 최상위(priority={top})")

    order = sorted(topics.items(), key=lambda kv: kv[1].get("priority", 0))
    print("    우선순위(낮→높): " + " < ".join(
        f"{n}({c['priority']})" for n, c in order))
    print(f"    {'✅' if ok else '❌'} input {len(topics)}개 / lock {len(locks)}개")
    return ok


# /cmd_vel 로 "발행"하는 패턴 (구독은 제외)
_PUB_PATTS = [
    re.compile(r"create_publisher\s*\(\s*Twist[^)]*?['\"]/?cmd_vel['\"]"),
    re.compile(r"create_publisher\s*\([^)]*?['\"]/cmd_vel['\"]"),
]


def check_code(src):
    print(f"\n[B] /cmd_vel 직접 발행 노드 스캔: {src}")
    offenders = []
    for fp in glob.glob(os.path.join(src, "**", "*.py"), recursive=True):
        if "/tools/" in fp:
            continue
        with open(fp, encoding="utf-8", errors="ignore") as f:
            txt = f.read()
        for i, line in enumerate(txt.splitlines(), 1):
            if "create_subscription" in line:        # 구독은 정상(출력 소비)
                continue
            if any(p.search(line) for p in _PUB_PATTS):
                offenders.append((fp, i, line.strip()))
    if not offenders:
        print("    ✅ /cmd_vel 직접 발행 노드 없음 (모두 input 토픽 경유로 추정)")
        return True
    print("    ❌ 아래 노드가 /cmd_vel 로 직접 발행 → mux input 토픽으로 변경 필요:")
    for fp, i, line in offenders:
        rel = os.path.relpath(fp, src)
        print(f"       {rel}:{i}  {line}")
    print("       (예: cow_tail_seek → tail_search/cmd_vel, cow_visual_servo → visual_servo/cmd_vel)")
    return False


def main():
    ap = argparse.ArgumentParser()
    _here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    ap.add_argument("--config", default=os.path.join(_here, "config", "twist_mux.yaml"))
    ap.add_argument("--src", default=_here)
    a = ap.parse_args()

    print("=" * 64)
    print(" twist_mux 우선순위 중재 검증")
    print("=" * 64)
    a_ok = check_config(a.config)
    b_ok = check_code(a.src)
    print("\n" + "=" * 64)
    if a_ok and b_ok:
        print(" 결과: ✅ twist_mux 중재 정상 — /cmd_vel 충돌 제거됨")
    else:
        print(" 결과: ❌ 위 항목 수정 필요 (config 또는 직접발행 노드)")
    print("=" * 64)
    sys.exit(0 if (a_ok and b_ok) else 1)


if __name__ == "__main__":
    main()
