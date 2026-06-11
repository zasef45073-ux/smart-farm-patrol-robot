# Copyright (c) 2024, Smart Farm Patrol Project - 꼬마 로봇 두리
# SPDX-License-Identifier: Apache-2.0
"""
arm_mass.py
===========
**팔 무게 가볍게(massless 트릭)** — 무팔 강건 보행정책 재사용용.

팔 링크 질량·관성을 거의 0(예: 0.01kg)으로 낮추면 로봇 동역학이 "무팔 Spot"과
같아져, 무팔 보행정책이 그대로 안정 보행(재학습 0). 팔은 위치/PID 로 따로 제어.

⚠️ 질량 정확히 0 금지(PhysX 불안정) → 작은 값 + 관성도 비례 축소.
순수 헬퍼(인덱스·질량/관성 계산)는 테스트 대상, Isaac 적용은 apply_light_arm(가드).
"""

ARM_PREFIX = "arm0"   # Spot 팔 링크 접두사 (arm0_link_sh0 ...)


def arm_body_indices(body_names, prefix=ARM_PREFIX):
    """전체 바디명 → 팔 링크 인덱스."""
    return [i for i, n in enumerate(body_names)
            if n.startswith(prefix) or "arm" in n.lower()]


def lighten_masses(masses, idxs, light_kg):
    """팔 인덱스 질량을 light_kg 로 교체한 새 리스트."""
    out = [float(m) for m in masses]
    for i in idxs:
        out[i] = float(light_kg)
    return out


def inertia_ratio(orig_mass, light_kg):
    """질량 축소 비율 = light_kg/orig_mass (관성도 같은 비율로 축소, 0 보호)."""
    orig_mass = float(orig_mass)
    if orig_mass <= 0.0:
        return 1.0
    return float(light_kg) / orig_mass


def apply_light_arm(robot, light_kg, prefix=ARM_PREFIX):
    """Isaac Lab Articulation 의 팔 링크 질량/관성을 런타임에 낮춘다(가드).

    Returns: 변경한 링크 수(실패/비활성 0).
    """
    try:
        import torch
        names = list(robot.body_names)
        idxs = arm_body_indices(names, prefix)
        if not idxs:
            return 0
        view = robot.root_physx_view
        m = view.get_masses().clone()
        inert = view.get_inertias().clone()
        for i in idxs:
            r = inertia_ratio(float(m[0, i]), light_kg)
            m[:, i] = float(light_kg)
            inert[:, i, :] *= r
        ids = torch.arange(m.shape[0])
        view.set_masses(m, ids)
        view.set_inertias(inert, ids)
        return len(idxs)
    except Exception as e:  # noqa: BLE001
        print(f"  ⚠️ 팔 경량화 실패(무시): {e}")
        return 0
