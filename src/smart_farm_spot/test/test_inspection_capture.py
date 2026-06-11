# Copyright (c) 2024, Smart Farm Patrol Project - 꼬마 로봇 두리
# SPDX-License-Identifier: Apache-2.0
"""검사 촬영 기록 단위 테스트 (cv2/Isaac 불요)."""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from inspection_capture import build_record, capture_paths, hotspot_summary  # noqa: E402


def test_capture_paths_format():
    p = capture_paths("/tmp/insp", track_id=3, frame_id=42)
    assert p["rgb"] == "/tmp/insp/cow3_00042_rgb.png"
    assert p["depth"].endswith("cow3_00042_depth.npy")
    assert p["meta"].endswith("cow3_00042_meta.json")


def test_hotspot_summary_none():
    assert hotspot_summary(None) is None
    assert hotspot_summary({}) is None


def test_hotspot_summary_extracts_fields():
    s = hotspot_summary({"x": 1, "y": 2, "w": 3, "h": 4, "cx": 10.44, "cy": 20.55, "area": 512.7})
    assert s == {"cx": 10.4, "cy": 20.6, "area": 512.7}


def test_build_record_core_fields():
    r = build_record(5, (3.2, -1.1), "/o/a_rgb.png", "/o/a_depth.npy", ts=123.0)
    assert r["track_id"] == 5
    assert r["cow_xy"] == [3.2, -1.1]
    assert r["rgb"].endswith("_rgb.png")
    assert r["timestamp"] == 123.0
    assert r["status"] == "captured"


def test_build_record_diagnosis_is_cloud_pending():
    # 판정은 항상 클라우드(아마존) 몫
    r = build_record(0, (0, 0), "a", "b", ts=1.0)
    assert r["diagnosis"] == "pending_cloud"
    assert r["hotspot"] is None


def test_build_record_includes_hotspot():
    r = build_record(1, (1, 1), "a", "b",
                     hotspot={"cx": 100.0, "cy": 50.0, "area": 800.0}, ts=1.0)
    assert r["hotspot"] == {"cx": 100.0, "cy": 50.0, "area": 800.0}
