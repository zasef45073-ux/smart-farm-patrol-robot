"""
유방염 검출 - HSV 빨간색 비율 판정
빨간 픽셀 비율이 임계치를 넘으면 유방염으로 판정
"""

import cv2
import numpy as np
import sys


def _red_ratio(hsv: np.ndarray) -> float:
    mask = (
        cv2.inRange(hsv, np.array([0,   70, 50]), np.array([10,  255, 255])) |
        cv2.inRange(hsv, np.array([170, 70, 50]), np.array([180, 255, 255]))
    )
    return np.count_nonzero(mask) / mask.size * 100


def detect_mastitis_array(img: np.ndarray, threshold_pct: float = 0.5) -> dict:
    """
    numpy BGR 배열을 받아 유방염 여부 반환 (파일 I/O 없음).

    Returns:
        {"detected": bool, "ratio": float}
    """
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    ratio = _red_ratio(hsv)
    return {"detected": ratio >= threshold_pct, "ratio": ratio}


def detect_mastitis(image_path: str, threshold_pct: float = 0.5) -> bool:
    """
    이미지 파일 경로를 받아 유방염 여부 반환.

    Args:
        image_path: 이미지 경로
        threshold_pct: 빨간색 비율 임계치 (기본 0.5%)

    Returns:
        True = 유방염, False = 정상
    """
    img = cv2.imread(image_path)
    if img is None:
        raise FileNotFoundError(f"이미지를 읽을 수 없습니다: {image_path}")
    result = detect_mastitis_array(img, threshold_pct)
    detected = result["detected"]
    print(f"빨간비율: {result['ratio']:.2f}% | 임계치: {threshold_pct}% | "
          f"{'🔴 유방염' if detected else '🟢 정상'}")
    return detected


if __name__ == "__main__":
    import os

    if len(sys.argv) < 2:
        print("사용법: python detect_mastitis.py image.jpg [임계치%]")
        print("        python detect_mastitis.py ./images/ [임계치%]")
        sys.exit(1)

    path = sys.argv[1]
    thr = float(sys.argv[2]) if len(sys.argv) > 2 else 0.5

    if os.path.isdir(path):
        for f in sorted(os.listdir(path)):
            if f.lower().endswith((".jpg", ".jpeg", ".png", ".bmp")):
                print(f"[{f}] ", end="")
                detect_mastitis(os.path.join(path, f), thr)
    else:
        detect_mastitis(path, thr)
