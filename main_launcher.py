import subprocess
import time
import json
import paho.mqtt.client as mqtt
import os
import threading

# MQTT 설정
MQTT_BROKER = "10.10.30.62"
MQTT_PORT = 31883
DOCK_TRIGGER_TOPIC = "/dmbot/v1/dn/navigation/gotocharger"
UNDOCK_TRIGGER_TOPIC = "/dmbot/v1/dn/navigation/gotoundock"

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

# 도킹 상태 플래그 (중복 실행 방지)
dock_lock = threading.Lock()
dock_in_progress = False

def handle_docking(payload):
    global dock_in_progress
    with dock_lock:
        if dock_in_progress:
            print("[도킹 진행 중] 새로운 도킹 명령 무시")
            return
        dock_in_progress = True

    def docking_thread():
        global dock_in_progress
        try:
            tag_id = int(payload["tag_id"])
            print(f"[도킹 시퀀스 시작] 마커 ID: {tag_id}")

            result = subprocess.run(["python", "first_stage.py", str(tag_id)], cwd=SCRIPT_DIR)

            if result.returncode == 0:
                print("[first_stage.py] 성공 → second_stage.py 실행")
                time.sleep(1)
                subprocess.run(["python", "second_stage.py", str(tag_id)], cwd=SCRIPT_DIR)
            elif result.returncode == 1:
                print(f"[first_stage.py] 마커 ID {tag_id} 감지 실패 → 종료")
            else:
                print(f"[first_stage.py] 예외 종료 코드: {result.returncode}")
        except Exception as e:
            print(f"[도킹 처리 에러] {e}")
        finally:
            with dock_lock:
                dock_in_progress = False

    threading.Thread(target=docking_thread, daemon=True).start()

def handle_undocking(payload):
    try:
        tag_id = int(payload["tag_id"])
        req_id = payload.get("req_id", "undock123")
        print(f"[언도킹 시퀀스 시작] 마커 ID: {tag_id}, req_id: {req_id}")

        subprocess.Popen(["python", "undocking.py", str(tag_id), req_id], cwd=SCRIPT_DIR)

    except Exception as e:
        print(f"[언도킹 처리 에러] {e}")

def on_mqtt_message(client, userdata, msg):
    try:
        payload = json.loads(msg.payload.decode())
        print(f"[MQTT 수신] {payload}")

        if msg.topic == DOCK_TRIGGER_TOPIC:
            handle_docking(payload)
        elif msg.topic == UNDOCK_TRIGGER_TOPIC:
            handle_undocking(payload)

    except Exception as e:
        print(f"[MQTT 메시지 처리 오류] {e}")

mqtt_client = mqtt.Client()
mqtt_client.on_message = on_mqtt_message
mqtt_client.connect(MQTT_BROKER, MQTT_PORT)
mqtt_client.subscribe(DOCK_TRIGGER_TOPIC)
mqtt_client.subscribe(UNDOCK_TRIGGER_TOPIC)
mqtt_client.loop_forever()


'''
여러번 실행 불가능
undocking 있는 버전


import subprocess
import time
import json
import paho.mqtt.client as mqtt
import os

# MQTT 설정
MQTT_BROKER = "10.10.30.62"
MQTT_PORT = 31883
DOCK_TRIGGER_TOPIC = "/dmbot/v1/dn/navigation/gotocharger"
UNDOCK_TRIGGER_TOPIC = "/dmbot/v1/dn/navigation/gotoundock"

# 현재 스크립트 위치
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

def handle_docking(payload):
    try:
        tag_id = int(payload["tag_id"])
        print(f"[도킹 시퀀스 시작] 마커 ID: {tag_id}")

        result = subprocess.run(["python", "first_stage.py", str(tag_id)], cwd=SCRIPT_DIR)

        if result.returncode == 0:
            print("[first_stage.py] 성공 → second_stage.py 실행")
            time.sleep(1)
            subprocess.run(["python", "second_stage.py", str(tag_id)], cwd=SCRIPT_DIR)
        elif result.returncode == 1:
            print(f"[first_stage.py] 마커 ID {tag_id} 감지 실패 → 종료")
        else:
            print(f"[first_stage.py] 예외 종료 코드: {result.returncode}")

    except Exception as e:
        print(f"[도킹 처리 에러] {e}")

def handle_undocking(payload):
    try:
        tag_id = int(payload["tag_id"])
        req_id = payload.get("req_id", "undock123")
        print(f"[언도킹 시퀀스 시작] 마커 ID: {tag_id}, req_id: {req_id}")

        # tag_id와 req_id를 인자로 전달
        subprocess.Popen(["python", "undocking.py", str(tag_id), req_id], cwd=SCRIPT_DIR)

    except Exception as e:
        print(f"[언도킹 처리 에러] {e}")

def on_mqtt_message(client, userdata, msg):
    try:
        payload = json.loads(msg.payload.decode())
        print(f"[MQTT 수신] {payload}")

        if msg.topic == DOCK_TRIGGER_TOPIC:
            handle_docking(payload)
        elif msg.topic == UNDOCK_TRIGGER_TOPIC:
            handle_undocking(payload)

    except Exception as e:
        print(f"[MQTT 메시지 처리 오류] {e}")

mqtt_client = mqtt.Client()
mqtt_client.on_message = on_mqtt_message
mqtt_client.connect(MQTT_BROKER, MQTT_PORT)
mqtt_client.subscribe(DOCK_TRIGGER_TOPIC)
mqtt_client.subscribe(UNDOCK_TRIGGER_TOPIC)
mqtt_client.loop_forever()


'''
'''
여러번 실행가능, undocking 없는 버전
import subprocess
import time
import json
import paho.mqtt.client as mqtt
import os

# MQTT 설정
MQTT_BROKER = "10.10.30.62"
MQTT_PORT = 31883
DOCK_TRIGGER_TOPIC = "/dmbot/v1/dn/navigation/gotocharger"

# 현재 스크립트 위치
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

def on_mqtt_message(client, userdata, msg):
    try:
        payload = json.loads(msg.payload.decode())
        print(f"[MQTT 수신] {payload}")

        if "tag_id" in payload:
            tag_id = int(payload["tag_id"])
            print(f"[도킹 시퀀스 시작] 마커 ID: {tag_id}")

            # first_stage.py 실행
            result = subprocess.run(
                ["python", "first_stage.py", str(tag_id)],
                cwd=SCRIPT_DIR
            )

            # 종료 코드 확인
            if result.returncode == 0:
                print("[first_stage.py] 마커 ID 일치 및 정렬 성공 → second_stage.py 실행")
                time.sleep(1)
                subprocess.run(["python", "second_stage.py", str(tag_id)], cwd=SCRIPT_DIR)
            elif result.returncode == 1:
                print(f"[first_stage.py] 2초 내 ID {tag_id} 마커 감지 실패 → second_stage.py 실행하지 않음")
            else:
                print(f"[first_stage.py] 기타 종료 코드 ({result.returncode}) → 실행 중단")

    except Exception as e:
        print(f"[에러] MQTT 메시지 처리 오류: {e}")

# MQTT 설정 및 시작
mqtt_client = mqtt.Client()
mqtt_client.on_message = on_mqtt_message
mqtt_client.connect(MQTT_BROKER, MQTT_PORT)
mqtt_client.subscribe(DOCK_TRIGGER_TOPIC)
mqtt_client.loop_forever()
'''