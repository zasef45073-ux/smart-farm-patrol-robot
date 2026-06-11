"""
detect_mastitis 이미지 분석 로직 목업 테스트
실행: python test_mastitis.py
"""

import numpy as np
from detect_mastitis import detect_mastitis_array

THR = 0.5  # 임계치 (%)

G     = "\033[92m"
R     = "\033[91m"
RESET = "\033[0m"


def make_solid(bgr: tuple) -> np.ndarray:
    return np.full((100, 100, 3), bgr, dtype=np.uint8)


def make_patch(red_pct: float) -> np.ndarray:
    """흰 바탕에 빨간 픽셀을 red_pct% 비율로 무작위 배치"""
    img  = np.full((100, 100, 3), 240, dtype=np.uint8)
    n    = max(1, int(10000 * red_pct / 100))
    idxs = np.random.choice(10000, n, replace=False)
    img.reshape(-1, 3)[idxs] = [0, 0, 200]
    return img


CASES = [
    # (설명,                         이미지,                    기대 결과)
    ("순수 빨간 이미지",              make_solid((0, 0, 200)),   True ),
    ("순수 초록 이미지",              make_solid((0, 200, 0)),   False),
    ("순수 파란 이미지",              make_solid((200, 0, 0)),   False),
    ("흰색 이미지",                   make_solid((240,240,240)), False),
    ("탁한 갈색 (채도 낮아 미감지)",   make_solid((50, 50, 60)), False),
    ("빨간 1.0% 패치 — 임계치 초과",  make_patch(1.0),           True ),
    ("빨간 0.3% 패치 — 임계치 미만",  make_patch(0.3),           False),
]


def run():
    print(f"\n{'='*60}")
    print(f"  detect_mastitis 목업 테스트  (임계치 {THR}%)")
    print(f"{'='*60}")

    passed = 0
    for name, img, expected in CASES:
        result  = detect_mastitis_array(img, threshold_pct=THR)
        ok      = result["detected"] == expected
        mark    = f"{G}PASS{RESET}" if ok else f"{R}FAIL{RESET}"
        outcome = "🔴 유방염" if result["detected"] else "🟢 정상"
        note    = "" if ok else f"  {R}← 기대: {'유방염' if expected else '정상'}{RESET}"
        print(f"  [{mark}]  {name:<30}  비율={result['ratio']:5.2f}%  {outcome}{note}")
        if ok:
            passed += 1

    print(f"{'─'*60}")
    color = G if passed == len(CASES) else R
    print(f"  결과: {color}{passed}/{len(CASES)} 통과{RESET}\n")
    return passed == len(CASES)


if __name__ == "__main__":
    import sys
    sys.exit(0 if run() else 1)
