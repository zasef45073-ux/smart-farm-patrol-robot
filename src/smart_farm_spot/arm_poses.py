# Copyright (c) 2024, Smart Farm Patrol Project - 꼬마 로봇 두리
# SPDX-License-Identifier: Apache-2.0
"""
arm_poses.py
============
**Spot 팔 자세 정의 + 전환/한계 헬퍼** — 순수 로직(torch/Isaac 불요, 학습 불요).

전제: 무팔 보행 정책 + **무게 거의 0인 팔**(CoM 외란 없음) → 팔은 위치제어로 자유 배치.
팔은 다리 정책 obs/action 밖(별도 position target). 이 모듈은 그 target 값을 만든다.

자세:
  · stow         : 등쪽 수납(주행 중 안정)
  · open         : 펼침(손 카메라가 소를 향함)
  · inspect_udder: **유방 하부 검사** — 팔을 앞-아래로 내리고 손목을 위로 틸트해
                   손끝 RGB-D 카메라가 **배 밑에서 위(유방)를 보게** 한다.

⚠️ 각도 기본값은 **in-sim 튜닝 시작점**이다(관절 부호/한계는 로봇 USD 의존).
   SF_INSP_* env 로 오버라이드하며 시뮬에서 카메라 시야를 보고 맞춘다.
"""
import os

# Spot 팔 6관절 (순서 고정)
ARM_JOINTS = ["arm0_sh0", "arm0_sh1", "arm0_el0", "arm0_el1", "arm0_wr0", "arm0_wr1"]

# Spot 다리 관절 접미사: _hx(고관절 외전) / _hy(피치) / _kn(무릎)
LEG_SUFFIXES = ("_hx", "_hy", "_kn")


def _f(env, default):
    return float(os.environ.get(env, str(default)))


def stow_pose():
    """등쪽 수납 — scenario.py 기존값과 동일."""
    return {"arm0_sh0": _f("SF_STOW_SH0", 0.0), "arm0_sh1": _f("SF_STOW_SH1", -3.05),
            "arm0_el0": _f("SF_STOW_EL0", 3.05), "arm0_el1": 0.0,
            "arm0_wr0": 0.0, "arm0_wr1": 0.0}


def open_pose():
    """전방 펼침 — 손 카메라가 소를 향함."""
    return {"arm0_sh0": _f("SF_OPEN_SH0", 0.0), "arm0_sh1": _f("SF_OPEN_SH1", -0.3),
            "arm0_el0": _f("SF_OPEN_EL0", 0.0), "arm0_el1": 0.0,
            "arm0_wr0": _f("SF_OPEN_WR0", 0.6), "arm0_wr1": 0.0}


def inspect_udder_pose():
    """유방 하부 검사 — 손끝 카메라가 배 밑에서 위(유방)를 보게.

    어깨 내림(sh1) + 팔꿈치 굽힘(el0)으로 손을 배 밑으로, 손목(wr0)으로 카메라 상향 틸트.
    (기본값=시작점. 시뮬에서 유방이 화면에 들어오게 SF_INSP_* 로 미세조정.)
    """
    return {"arm0_sh0": _f("SF_INSP_SH0", 0.0),     # 좌우 정렬(정면)
            "arm0_sh1": _f("SF_INSP_SH1", -0.6),    # 어깨 아래로(팔 낮춤)
            "arm0_el0": _f("SF_INSP_EL0", 1.2),     # 팔꿈치 굽혀 손을 배 밑으로
            "arm0_el1": _f("SF_INSP_EL1", 0.0),
            "arm0_wr0": _f("SF_INSP_WR0", -1.2),    # 손목 위로 틸트(카메라 상향=유방)
            "arm0_wr1": _f("SF_INSP_WR1", 0.0)}


# ── 헬퍼 (순수·테스트 대상) ───────────────────────────────────────────
def pose_vector(pose, joint_names=ARM_JOINTS):
    """pose dict → joint_names 순서 리스트 (없는 관절은 0.0)."""
    return [float(pose.get(n, 0.0)) for n in joint_names]


def arm_indices(all_joint_names, arm_joints=ARM_JOINTS):
    """articulation 전체 관절명 → 팔 관절의 인덱스 리스트 (없는 건 건너뜀)."""
    return [i for i, n in enumerate(all_joint_names) if n in arm_joints]


def clamp_to_limits(vec, limits):
    """각 관절을 (lo, hi) 한계로 클램프. limits=[(lo,hi),...]."""
    out = []
    for v, (lo, hi) in zip(vec, limits):
        out.append(lo if v < lo else (hi if v > hi else v))
    return out


def lerp_pose(start_vec, target_vec, t):
    """start→target 선형보간(부드러운 전환). t∈[0,1] 로 클램프."""
    t = 0.0 if t < 0.0 else (1.0 if t > 1.0 else t)
    return [s * (1.0 - t) + g * t for s, g in zip(start_vec, target_vec)]


def transition_progress(step, start_step, duration):
    """전환 진행도 t=(step-start_step)/duration, [0,1] 클램프. duration<=0 이면 1."""
    if duration <= 0:
        return 1.0
    t = (step - start_step) / float(duration)
    return 0.0 if t < 0.0 else (1.0 if t > 1.0 else t)


# ── 앉기(sit/crouch) — 다리 자세 ──────────────────────────────────────
# "후방 이동 후 앉아서 검사": 다리를 정책→위치제어로 전환해 몸을 낮춘다.
#   · CoM 낮아짐 + 베이스 지면 근접 → 매우 안정(팔 검사 외란 흡수)
#   · 카메라가 유방 높이에 가까워짐
# ⚠️ 정책이 STAND_Z(0.62)를 유지하므로, 앉으려면 INSPECT 동안 다리를 위치제어로
#    바꿔야 한다(정책 leg action 중지). 목표 z 는 FALL_Z(0.30) 위(예: ~0.40)로
#    유지해 넘어짐 복구(force-stand)가 오작동하지 않게 한다.
def leg_indices(all_joint_names):
    """articulation 전체 관절명 → 다리 관절 인덱스."""
    return [i for i, n in enumerate(all_joint_names) if n.endswith(LEG_SUFFIXES)]


def sit_offsets():
    """앉기 — stand 기본값에 더할 접미사별 오프셋(접어 몸 낮춤).

    부호/크기는 USD 관절 규약 의존 → in-sim 튜닝(SF_SIT_*). 기본값은 시작점.
    """
    return {"_hy": _f("SF_SIT_HY", 0.6),    # 고관절 피치 굽힘(엉덩이 내림)
            "_kn": _f("SF_SIT_KN", -0.9),   # 무릎 더 굽힘(몸 낮춤)
            "_hx": _f("SF_SIT_HX", 0.0)}    # 외전(보통 0; 필요시 지지면 확대)


def sit_leg_targets(default_positions, joint_names, offsets=None):
    """stand 기본 관절위치 + 다리 접미사 오프셋 → 앉기 target(다리만 변경).

    Args:
        default_positions: stand 시 전체 관절위치(리스트)
        joint_names:        같은 순서의 전체 관절명
        offsets:            접미사→오프셋(미지정 시 sit_offsets())
    Returns:
        앉기 target 전체 벡터(다리 관절만 오프셋 적용, 그 외 동일)
    """
    offsets = offsets or sit_offsets()
    out = list(default_positions)
    for i, n in enumerate(joint_names):
        for suf, d in offsets.items():
            if n.endswith(suf):
                out[i] = default_positions[i] + d
    return out
