# Copyright (c) 2024, Smart Farm Patrol Project - 꼬마 로봇 두리
# SPDX-License-Identifier: Apache-2.0
"""
setup_semantics.py
==================
Isaac Sim 내부 객체 인식(Ground-Truth Bounding Box)을 위한 시맨틱 라벨링.

[최적화 핵심]
  무거운 YOLO/딥러닝 모델을 엣지에서 돌리지 않고,
  Isaac Sim의 Synthetic Data 파이프라인이 제공하는 "정답 Bounding Box"를 사용.
  → 추론 연산 비용 0, 정확도 100% (SITL 데모 최적)
  → 소의 유방/다리 메쉬에 시맨틱 클래스 라벨만 붙이면
    카메라가 바라볼 때 bbox_2d_tight 어노테이터가 자동으로 박스를 출력.

  실제 질병 판독(유방염/파행)은 클라우드에서 수행.
  엣지의 "객체 위치 탐지"만 Isaac Sim 정답 데이터로 대체.

라벨 클래스:
  udder   - 소 유방 (유방염 관찰 대상)
  leg     - 소 뒷다리 (파행 관찰 대상)
  cow     - 소 전체 (접근/회피 기준)

사용법 (Script Editor):
  from setup_semantics import tag_semantics, auto_tag_by_keyword
  # 직접 prim 지정
  tag_semantics("/World/Cow_01/Udder", "udder")
  tag_semantics("/World/Cow_01/RearLeg_L", "leg")
  # 또는 이름 키워드로 자동 탐색
  auto_tag_by_keyword()
"""

# Isaac Sim 5.1 시맨틱 유틸 (네임스페이스 폴백 포함)
try:
    from isaacsim.core.utils.semantics import add_update_semantics, remove_all_semantics
except ImportError:
    from omni.isaac.core.utils.semantics import add_update_semantics, remove_all_semantics

from isaacsim.core.utils.stage import get_current_stage  # noqa
import omni.usd


# ── 라벨 매핑 (키워드 → 시맨틱 클래스) ──────────────────────────────
KEYWORD_TO_CLASS = {
    "udder":   "udder",   # 유방
    "teat":    "udder",
    "mammary": "udder",
    "leg":     "leg",     # 다리
    "lleg":    "leg",
    "uleg":    "leg",
    "shank":   "leg",
    "hock":    "leg",
    "cow":     "cow",     # 소 전체
    "cattle":  "cow",
    "bovine":  "cow",
}


def tag_semantics(prim_path: str, class_name: str) -> bool:
    """
    지정한 prim에 시맨틱 클래스 라벨을 부여.

    Args:
        prim_path:  대상 prim 경로 (예: "/World/Cow_01/Udder")
        class_name: 시맨틱 클래스 (udder / leg / cow)

    Returns:
        성공 여부
    """
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(prim_path)
    if not prim or not prim.IsValid():
        print(f"  ❌ prim 없음: {prim_path}")
        return False

    add_update_semantics(prim, semantic_label=class_name, type_label="class")
    print(f"  ✅ 라벨 부여: {prim_path}  →  '{class_name}'")
    return True


def auto_tag_by_keyword(root_path: str = "/World") -> int:
    """
    씬 전체를 순회하며 prim 이름에 키워드가 포함되면 자동 라벨링.

    Args:
        root_path: 탐색 시작 경로 (기본 /World)

    Returns:
        라벨링된 prim 개수
    """
    stage = omni.usd.get_context().get_stage()
    root = stage.GetPrimAtPath(root_path)
    if not root or not root.IsValid():
        print(f"  ❌ 루트 경로 없음: {root_path}")
        return 0

    count = 0
    print(f"[auto_tag] '{root_path}' 하위 키워드 탐색...")
    for prim in stage.Traverse():
        name_lower = prim.GetName().lower()
        for keyword, class_name in KEYWORD_TO_CLASS.items():
            if keyword in name_lower:
                add_update_semantics(prim, semantic_label=class_name, type_label="class")
                print(f"  ✅ {str(prim.GetPath()):<50} → '{class_name}' (키워드: {keyword})")
                count += 1
                break

    if count == 0:
        print("  ⚠️  키워드 일치 prim 없음. tag_semantics()로 직접 지정하세요.")
        print(f"     예: tag_semantics('{root_path}/YourCow/Udder', 'udder')")
    else:
        print(f"\n[auto_tag] 총 {count}개 prim 라벨링 완료")
    return count


def clear_semantics(prim_path: str) -> bool:
    """지정 prim의 모든 시맨틱 라벨 제거."""
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(prim_path)
    if not prim or not prim.IsValid():
        return False
    remove_all_semantics(prim, recursive=True)
    print(f"  🧹 라벨 제거: {prim_path}")
    return True


def list_semantics(root_path: str = "/World") -> None:
    """현재 씬에 부여된 모든 시맨틱 라벨 출력."""
    stage = omni.usd.get_context().get_stage()
    print(f"[list] '{root_path}' 하위 시맨틱 라벨:")
    found = False
    for prim in stage.Traverse():
        for prop in prim.GetProperties():
            if "semantic" in prop.GetName().lower() and "Type" not in prop.GetName():
                val = prim.GetAttribute(prop.GetName()).Get()
                if val:
                    print(f"  {str(prim.GetPath()):<50} {prop.GetName()} = {val}")
                    found = True
    if not found:
        print("  (라벨 없음)")


if __name__ == "__main__":
    print("=" * 60)
    print("  소 객체 시맨틱 라벨링 (Isaac Sim 내부 객체 인식)")
    print("=" * 60)
    print("""
  사용 예:
    tag_semantics("/World/Cow_01/Udder", "udder")
    tag_semantics("/World/Cow_01/RearLeg_L", "leg")
    auto_tag_by_keyword("/World")     # 이름 키워드 자동 탐색
    list_semantics()                  # 현재 라벨 확인
""")
    auto_tag_by_keyword("/World")
