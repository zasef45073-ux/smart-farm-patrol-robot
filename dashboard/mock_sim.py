"""
목업 시뮬레이터 — 로봇 없이 전체 흐름 테스트용

실행: python mock_sim.py
      python mock_sim.py --url http://localhost:5000  # 서버 주소 변경
      python mock_sim.py --fast                       # 빠른 모드 (0.5초 간격)
      python mock_sim.py --camera                     # 카메라 피드도 시뮬레이션
"""

import argparse
import time
import random
import requests
import threading
from datetime import datetime

parser = argparse.ArgumentParser()
parser.add_argument('--url', default='http://localhost:5000')
parser.add_argument('--fast', action='store_true')
parser.add_argument('--camera', action='store_true')
args = parser.parse_args()

BASE    = args.url
DELAY   = 0.5 if args.fast else 3.0

COWS     = ['1번 소', '2번 소', '3번 소', '4번 소', '5번 소']
DISEASES = {
    '위험': ['구제역 의심', '유방염 (중증)', '호흡기 감염 (중증)', '기침 반복', '식욕 저하', '보행 이상'],
    '정상': ['정상', '정상', '정상'],
}
COURSES  = ['A코스', 'B코스', 'C코스']


def ts():
    return datetime.now().strftime('%Y-%m-%d %H:%M:%S')


def post(path, body):
    try:
        r = requests.post(BASE + path, json=body, timeout=3)
        return r.status_code == 200
    except Exception as e:
        print(f'  [오류] {path} → {e}')
        return False


def seed_cattle():
    """소 기본 정보 등록 (이미 있으면 무시)"""
    print('[시드] 소 정보 등록 중...')
    existing = []
    try:
        existing = [c['reg_number'] for c in requests.get(BASE + '/api/edr/cattle', timeout=3).json()]
    except Exception:
        pass

    for i, cow in enumerate(COWS, 1):
        if cow in existing:
            continue
        post('/api/edr/cattle', {
            'reg_number': cow,
            'weight': round(random.uniform(450, 650), 1),
            'birth_date': f'202{random.randint(1,3)}-{random.randint(1,12):02d}-{random.randint(1,28):02d}',
            'status': '정상',
        })
    print(f'  → {len(COWS)}마리 준비 완료')


def sim_detection():
    """감지 이벤트 1건 전송"""
    severity = random.choices(['위험', '정상'], weights=[1, 5])[0]
    cow      = random.choice(COWS)
    disease  = random.choice(DISEASES[severity])
    payload  = {'cow_id': cow, 'disease': disease, 'severity': severity, 'timestamp': ts()}
    ok = post('/api/save', payload)
    mark = '🔴' if severity == '위험' else '🟢'
    print(f'  {mark} 감지 전송 {"✓" if ok else "✗"} | {cow} | {disease} | {severity}')


def sim_robot_cycle():
    """순찰 사이클: idle → patrolling → idle"""
    course = random.choice(COURSES)
    print(f'\n[로봇] {course} 순찰 시작')
    post('/api/robot/status', {'status': 'patrolling', 'course': course})
    time.sleep(DELAY * 4)

    print(f'[로봇] {course} 순찰 완료 → 대기')
    post('/api/robot/status', {'status': 'idle', 'course': None})


def sim_camera_loop():
    """카메라 피드 시뮬레이션 — 색상 블록 JPEG을 0.5초마다 전송"""
    import io
    import base64
    try:
        import cv2
        import numpy as np
    except ImportError:
        print('[카메라] opencv-python 없음 — pip install opencv-python-headless')
        return

    print('[카메라] 피드 시뮬레이션 시작')
    colors = [(60, 120, 200), (40, 160, 80), (200, 80, 60), (160, 100, 200)]
    idx = 0
    while True:
        color = colors[idx % len(colors)]
        frame = np.full((480, 640, 3), color, dtype='uint8')
        label = f'MOCK CAMERA  {datetime.now().strftime("%H:%M:%S")}'
        cv2.putText(frame, label, (30, 240), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255,255,255), 2)
        _, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
        img_b64 = base64.b64encode(buf).decode()
        try:
            requests.post(BASE + '/api/camera', json={'image': img_b64}, timeout=2)
        except Exception:
            pass
        idx += 1
        time.sleep(0.5)


def main():
    print(f'목업 시뮬레이터 시작 (서버: {BASE}, 간격: {DELAY}s)')
    print('Ctrl+C 로 종료\n')

    # 서버 연결 확인
    try:
        requests.get(BASE + '/api/robot/status', timeout=3)
    except Exception:
        print(f'[오류] 서버에 연결할 수 없습니다: {BASE}')
        print('서버를 먼저 실행하세요: docker-compose up  또는  python server.py')
        return

    seed_cattle()

    if args.camera:
        threading.Thread(target=sim_camera_loop, daemon=True).start()

    print()

    cycle = 0
    while True:
        cycle += 1
        print(f'--- 사이클 {cycle} ({ts()}) ---')

        # 감지 2~4건 전송
        for _ in range(random.randint(2, 4)):
            sim_detection()
            time.sleep(DELAY * 0.3)

        # 5사이클마다 순찰 시뮬레이션
        if cycle % 5 == 0:
            sim_robot_cycle()

        print()
        time.sleep(DELAY)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('\n시뮬레이터 종료')
