import sys
import cv2
import cv2.aruco as aruco
import numpy as np
import time
import json
import paho.mqtt.client as mqtt

# 인자 처리
tag_id = int(sys.argv[1]) if len(sys.argv) > 1 else 0
req_id = sys.argv[2] if len(sys.argv) > 2 else "undock123"

# 설정
UNDOCK_SPEED = 0.05
UNDOCK_DURATION = 10
UNDOCK_TIMEOUT = 2.0

# MQTT 설정
MQTT_BROKER = "10.10.30.62"
MQTT_PORT = 31883
CMD_VEL_TOPIC = "/ros2/cmd_vel"

mqtt_client = mqtt.Client()
mqtt_client.connect(MQTT_BROKER, MQTT_PORT)
mqtt_client.loop_start()

def publish_cmd_vel(linear=0.0, angular=0.0):
    msg = {
        "req_id": req_id,
        "linear": linear,
        "angular": angular
    }
    mqtt_client.publish(CMD_VEL_TOPIC, json.dumps(msg))

# 카메라 설정
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
parameters = aruco.DetectorParameters()
marker_length = 0.06

camera_matrix = np.array([
    [481.9547616, 0., 311.91529054],
    [0., 484.45063462, 222.5327912],
    [0., 0., 1.]
], dtype=np.float64)
dist_coeffs = np.array([[-0.04661008, 0.08111488, -0.00012161, 0.00230337, -0.6278585]])

cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
start_time = time.time()

if tag_id == 0:
    print("[언도킹 시작] 마커 없이 즉시 전진")
    for _ in range(UNDOCK_DURATION):
        publish_cmd_vel(UNDOCK_SPEED, 0.0)
        time.sleep(1.0)
    publish_cmd_vel(0.0, 0.0)
    print("[언도킹 완료]")
else:
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        current_time = time.time()

        if ids is not None:
            for i in range(len(ids)):
                marker = int(ids[i][0])
                if marker == tag_id:
                    print(f"[언도킹 시작] 마커 ID {tag_id} 감지됨")
                    for _ in range(UNDOCK_DURATION):
                        publish_cmd_vel(UNDOCK_SPEED, 0.0)
                        time.sleep(1.0)
                    publish_cmd_vel(0.0, 0.0)
                    print("[언도킹 완료]")
                    cap.release()
                    cv2.destroyAllWindows()
                    mqtt_client.loop_stop()
                    mqtt_client.disconnect()
                    sys.exit(0)

        if current_time - start_time > UNDOCK_TIMEOUT:
            print(f"[타임아웃] {UNDOCK_TIMEOUT}초 내 ID {tag_id} 마커 미감지 → 언도킹 실패")
            break

        cv2.imshow("Undocking View", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

cap.release()
cv2.destroyAllWindows()
mqtt_client.loop_stop()
mqtt_client.disconnect()
