# Copyright (c) 2024, Smart Farm Patrol Project - 꼬마 로봇 두리
# SPDX-License-Identifier: Apache-2.0
"""
inspection_capture.py
=====================
**검사 촬영 기록** — 순수 로직(cv2/Isaac 불요). 파일 경로 + 클라우드 전송용 레코드 생성.

설계: 로봇은 **포즈 lock-on → RGB-D 촬영 + 열점(hotspot) 검출** 까지만 하고,
실제 유방염 **판정은 클라우드(아마존 서버)** 가 한다(setup_semantics: 질병 판독=클라우드).
→ 레코드의 diagnosis 는 항상 "pending_cloud".

scenario.py(Isaac)가 rgb/depth 를 실제 저장하고, 이 모듈로 경로·레코드를 만든다.
"""
import os
import time


def capture_paths(out_dir, track_id, frame_id):
    """소 track_id + frame_id 로 rgb/depth/meta 저장 경로 생성."""
    base = f"cow{int(track_id)}_{int(frame_id):05d}"
    return {"rgb": os.path.join(out_dir, base + "_rgb.png"),
            "depth": os.path.join(out_dir, base + "_depth.npy"),
            "meta": os.path.join(out_dir, base + "_meta.json")}


def hotspot_summary(bbox):
    """thermal_processor.get_hotspot_bbox 결과 → 요약(없으면 None)."""
    if not bbox:
        return None
    return {"cx": round(float(bbox["cx"]), 1),
            "cy": round(float(bbox["cy"]), 1),
            "area": round(float(bbox["area"]), 1)}


def build_record(track_id, cow_xy, rgb_path, depth_path, hotspot=None, ts=None):
    """클라우드 전송용 검사 레코드. 판정(diagnosis)은 클라우드 몫 → pending_cloud."""
    return {"track_id": int(track_id),
            "cow_xy": [round(float(cow_xy[0]), 3), round(float(cow_xy[1]), 3)],
            "rgb": rgb_path,
            "depth": depth_path,
            "hotspot": hotspot_summary(hotspot),
            "timestamp": time.time() if ts is None else ts,
            "status": "captured",
            "diagnosis": "pending_cloud"}
