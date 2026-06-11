import paho.mqtt.client as mqtt
import requests
import json
from datetime import datetime

BROKER     = "mosquitto"
PORT       = 1883
TOPIC      = "cattle/detection"
SERVER_URL = "http://web:5000/api/save"

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print(f"[연결성공] MQTT 브로커 연결됨 (토픽: {TOPIC})")
        client.subscribe(TOPIC)
    else:
        print(f"[연결실패] 오류 코드: {rc}")

def on_message(client, userdata, msg):
    try:
        payload = json.loads(msg.payload.decode("utf-8"))
        payload["timestamp"] = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        print(f"[수신] {payload}")

        # Flask API 호출 → 저장 + SSE 푸시 동시에 처리
        res = requests.post(SERVER_URL, json=payload)
        if res.status_code == 200:
            print(f"[저장완료] FastAPI로 전달 성공")
        else:
            print(f"[오류] FastAPI 응답: {res.status_code}")

    except json.JSONDecodeError:
        print(f"[오류] JSON 파싱 실패: {msg.payload}")
    except Exception as e:
        print(f"[오류] {e}")

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

print(f"[시작] MQTT 브로커({BROKER}:{PORT}) 연결 시도 중...")
client.connect(BROKER, PORT, keepalive=60)
client.loop_forever()