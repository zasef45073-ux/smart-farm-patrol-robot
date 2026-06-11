#!/usr/bin/env python3
# Copyright (c) 2024, Smart Farm Patrol Project - 꼬마 로봇 두리
# SPDX-License-Identifier: Apache-2.0
"""
check_warmstart.py
==================
**235 → 236 warm-start 검증 도구** (Isaac Lab/torch 환경에서 실행).

phase(위상) 1차원을 obs 에 추가해 기존 보행 정책에서 warm-start 할 때,
"기존 행동을 깨지 않고 시작하는지"를 학습 전에 확정한다.

검사 항목:
  [A] 첫 Linear 가중치 확장 — [512,235] → [512,236], 기존 235열 보존 + 새 phase열 0
  [B] bias / 이후 층(512→256→128→12) 변화 없음
  [C] observation normalizer (mean/std) 235 → 236 확장 (있으면)
  [D] 기능 등가성 — phase=0 인 236 obs 가 기존 235 정책과 동일 액션 산출 (★핵심)

지원 포맷:
  · TorchScript actor (.pt, torch.jit) — 배포 정책(policy.pt) 형식. forward 호출 가능 → [D] 가능
  · state_dict 체크포인트(.pt) — 학습 체크포인트. [A][B][C] 구조 검사

실행 예:
  python3 check_warmstart.py --old policy_235.pt --new policy_236.pt
  python3 check_warmstart.py --old old.pt --new new.pt --phase-index -1 --tol 1e-5
  # phase 를 obs 중간(예: index 48)에 끼웠다면 --phase-index 48
"""
import argparse
import sys

import numpy as np

try:
    import torch
except ImportError:
    print("[ERROR] torch 가 필요합니다. Isaac Lab venv 에서 실행하세요.")
    sys.exit(2)


# ── 로딩 헬퍼 ──────────────────────────────────────────────────────────
def load_actor(path):
    """(callable_or_None, params: dict[name->Tensor], normalizer: dict|None) 반환."""
    callable_fn, params, norm = None, {}, None
    try:
        m = torch.jit.load(path, map_location="cpu")
        m.eval()
        callable_fn = m
        params = {n: p.detach().cpu() for n, p in m.named_parameters()}
        buffers = {n: b.detach().cpu() for n, b in m.named_buffers()}
        norm = _extract_normalizer(buffers)
        return callable_fn, params, norm
    except Exception:
        pass
    # state_dict 체크포인트
    ckpt = torch.load(path, map_location="cpu")
    sd = None
    if isinstance(ckpt, dict):
        for k in ("model_state_dict", "actor_state_dict", "state_dict", "model"):
            if k in ckpt and isinstance(ckpt[k], dict):
                sd = ckpt[k]
                break
        if sd is None and all(torch.is_tensor(v) for v in ckpt.values()):
            sd = ckpt
    if sd is None:
        raise RuntimeError(f"{path}: state_dict 를 찾지 못함")
    params = {n: v.detach().cpu() for n, v in sd.items() if torch.is_tensor(v)}
    norm = _extract_normalizer(params)
    return None, params, norm


def _extract_normalizer(d):
    """running mean/var(또는 std) 버퍼 탐지 → {'mean':T, 'std':T} 또는 None."""
    mean = std = None
    for n, t in d.items():
        ln = n.lower()
        if mean is None and ln.endswith("mean") and t.ndim == 1:
            mean = t
        if std is None and (ln.endswith("std") or ln.endswith("var")) and t.ndim == 1:
            std = t if ln.endswith("std") else torch.sqrt(t + 1e-8)
    if mean is not None:
        return {"mean": mean, "std": std}
    return None


def _first_linear_weight(params):
    """입력 차원이 235 또는 236 인 첫 Linear 가중치 (name, Tensor)."""
    for n, t in params.items():
        if n.endswith("weight") and t.ndim == 2 and t.shape[1] in (235, 236):
            return n, t
    # 폴백: 가장 큰 in_features 2D
    cand = [(n, t) for n, t in params.items() if t.ndim == 2 and n.endswith("weight")]
    cand.sort(key=lambda x: x[1].shape[1], reverse=True)
    return cand[0] if cand else (None, None)


# ── 검사 ───────────────────────────────────────────────────────────────
def insert_phase(obs235, idx, value=0.0):
    """235 obs 에 phase 값을 idx 위치에 삽입 → 236. idx=-1 이면 맨 뒤."""
    n = obs235.shape[-1]
    k = n + 1 + idx if idx < 0 else idx   # -1 → n (맨 뒤)
    return np.insert(obs235, k, value, axis=-1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--old", required=True, help="기존 235 정책")
    ap.add_argument("--new", required=True, help="warm-start 한 236 정책")
    ap.add_argument("--phase-index", type=int, default=-1,
                    help="obs 내 phase 위치 (default -1 = 맨 뒤 append)")
    ap.add_argument("--n", type=int, default=512, help="기능검사 샘플 수")
    ap.add_argument("--tol", type=float, default=1e-4, help="등가성 허용오차")
    a = ap.parse_args()

    print("=" * 64)
    print(" 235 → 236 warm-start 검증")
    print("=" * 64)
    old_fn, old_p, old_norm = load_actor(a.old)
    new_fn, new_p, new_norm = load_actor(a.new)
    ok = True

    # [A] 첫 Linear 확장
    on, ow = _first_linear_weight(old_p)
    nn_, nw = _first_linear_weight(new_p)
    print(f"\n[A] 첫 Linear: old {on}={tuple(ow.shape)}  →  new {nn_}={tuple(nw.shape)}")
    if ow.shape[1] != 235 or nw.shape[1] != 236 or ow.shape[0] != nw.shape[0]:
        print("    ❌ 차원 기대 불일치 ([512,235]→[512,236] 아님)"); ok = False
    else:
        k = nw.shape[1] - 1 + a.phase_index + 1 if a.phase_index < 0 else a.phase_index
        kept_new = torch.cat([nw[:, :k], nw[:, k + 1:]], dim=1)
        new_col = nw[:, k]
        same = torch.allclose(kept_new, ow, atol=1e-6)
        zero = torch.allclose(new_col, torch.zeros_like(new_col), atol=1e-6)
        print(f"    기존 235열 보존: {'✅' if same else '❌ (열 위치/순서 의심)'}"
              f"   (max차이={float((kept_new-ow).abs().max()):.2e})")
        print(f"    새 phase열(idx={k}) 0초기화: {'✅' if zero else '❌ (0 아님 → 행동 깨짐)'}"
              f"   (max|col|={float(new_col.abs().max()):.2e})")
        ok = ok and same and zero

    # [B] 이후 층/bias 동일
    shared = [n for n in old_p if n in new_p and old_p[n].shape == new_p[n].shape]
    diffs = [n for n in shared if not torch.allclose(old_p[n], new_p[n], atol=1e-6)]
    print(f"\n[B] 동일형상 파라미터 {len(shared)}개 중 변경됨: {len(diffs)}개")
    if diffs:
        print(f"    ⚠️ 변경된 층: {diffs[:6]}{' ...' if len(diffs) > 6 else ''}")
        print("    (이후 층까지 바뀌었으면 단순 warm-start 아님 — 의도 확인)")

    # [C] normalizer 확장
    print("\n[C] observation normalizer")
    if old_norm and new_norm:
        om, nm = old_norm["mean"], new_norm["mean"]
        print(f"    mean: {tuple(om.shape)} → {tuple(nm.shape)}", end="  ")
        if nm.numel() == om.numel() + 1:
            kk = nm.numel() - 1 + a.phase_index + 1 if a.phase_index < 0 else a.phase_index
            kept = torch.cat([nm[:kk], nm[kk + 1:]])
            print("✅ 확장" if torch.allclose(kept, om, atol=1e-5) else "❌ 기존값 안 맞음")
        else:
            print("❌ +1 확장 아님 (normalizer 차원 확인 필요)"); ok = False
    elif old_norm or new_norm:
        print("    ⚠️ 한쪽만 normalizer 존재 — 학습/배포 정규화 불일치 위험");
    else:
        print("    (normalizer 버퍼 없음 — env 측 정규화면 스킵)")

    # [D] 기능 등가성 (★) — phase=0 이면 두 정책 액션 동일?
    print("\n[D] 기능 등가성 (phase=0 obs → 기존과 동일 액션)")
    if old_fn is None or new_fn is None:
        print("    ⏭  jit(forward 호출 가능) 정책이 아니라 스킵 — [A][C] 로 간접 보증")
    else:
        rng = np.random.default_rng(0)
        max_d = 0.0
        for _ in range(a.n):
            o235 = rng.standard_normal((1, 235)).astype(np.float32)
            o236 = insert_phase(o235, a.phase_index, 0.0).astype(np.float32)
            with torch.inference_mode():
                a_old = old_fn(torch.from_numpy(o235)).cpu().numpy()
                a_new = new_fn(torch.from_numpy(o236)).cpu().numpy()
            max_d = max(max_d, float(np.abs(a_old - a_new).max()))
        verdict = "✅ 동일(행동 보존)" if max_d <= a.tol else "❌ 어긋남(열 위치/0초기화/순서 확인)"
        print(f"    {a.n}회 비교 최대 액션차이={max_d:.2e} (tol={a.tol:.0e}) → {verdict}")
        ok = ok and max_d <= a.tol

    print("\n" + "=" * 64)
    print(" 결과:", "✅ warm-start 정상 — 학습 진행 OK" if ok
          else "❌ 문제 발견 — 위 항목 수정 후 재학습")
    print("=" * 64)
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
