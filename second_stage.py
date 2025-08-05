import sys
import cv2
import cv2.aruco as aruco
import numpy as np
import time
from collections import deque, defaultdict
import json
import paho.mqtt.client as mqtt

target_marker_id = int(sys.argv[1]) if len(sys.argv) > 1 else None
dock_enabled = True

# MQTT 설정
MQTT_BROKER = "10.10.30.62"
MQTT_PORT = 31883
CMD_VEL_TOPIC = "/ros2/cmd_vel"

mqtt_client = mqtt.Client()
mqtt_client.connect(MQTT_BROKER, MQTT_PORT)
mqtt_client.loop_start()

def publish_cmd_vel(linear=0.0, angular=0.0, req_id="dock"):
    msg = {
        "req_id": req_id,
        "linear": linear,
        "angular": angular
    }
    mqtt_client.publish(CMD_VEL_TOPIC, json.dumps(msg))

# 카메라 및 ArUco 설정
camera_matrix = np.array([
    [486.70775596, 0., 276.18339759],
    [0., 487.6043978 ,232.99005412],
    [0., 0., 1.]
], dtype=np.float64)

dist_coeffs = np.array([[0.0512068, -0.21021301, 0.00197381, -0.02546547, 0.09921895]])
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
parameters = aruco.DetectorParameters()
marker_length = 0.093

WINDOW_SIZE = 10
x_queues = defaultdict(lambda: deque(maxlen=WINDOW_SIZE))
z_queues = defaultdict(lambda: deque(maxlen=WINDOW_SIZE))
pitch_queues = defaultdict(lambda: deque(maxlen=WINDOW_SIZE))

PITCH_TOLERANCE = 5.0
PIXEL_TOLERANCE = 15
LINEAR_SPEED_1 = 0.03
ANGULAR_SPEED_1 = 0.03

LINEAR_SPEED_2 = 0.02
ANGULAR_SPEED_2 = 0.03

LINEAR_SPEED_3 = 0.01
ANGULAR_SPEED_3 = 0.02

CAMERA_OFFSET = 0.45
Z_THRESHOLD_1 = 0.5
Z_THRESHOLD_2 = 0.25
Z_STOP_THRESHOLD = 0.2

def rotationMatrixToEulerAngles(R):
    sy = np.sqrt(R[0, 0]**2 + R[1, 0]**2)
    singular = sy < 1e-6
    if not singular:
        x = np.arctan2(R[2, 1], R[2, 2])
        y = np.arctan2(-R[2, 0], sy)
        z = np.arctan2(R[1, 0], R[0, 0])
    else:
        x = np.arctan2(-R[1, 2], R[1, 1])
        y = np.arctan2(-R[2, 0], sy)
        z = 0
    return np.degrees([x, y, z])

cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
last_update_time = time.time()
update_interval = 1.0

while True:
    ret, frame = cap.read()
    if not ret or frame is None:
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    if ids is not None:
        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, marker_length, camera_matrix, dist_coeffs)

        for i in range(len(ids)):
            marker_id = int(ids[i][0])
            if not dock_enabled or marker_id != target_marker_id:
                continue

            rvec = rvecs[i][0]
            tvec = tvecs[i][0]
            x_cam = tvec[0]
            z_cam = tvec[2]

            rotation_matrix, _ = cv2.Rodrigues(rvec)
            pitch_deg = rotationMatrixToEulerAngles(rotation_matrix)[1]
            pitch_rad = np.radians(pitch_deg)
            x_base = x_cam + CAMERA_OFFSET * np.sin(pitch_rad)
            z_distance = z_cam

            # 픽셀 기준 마커 중심 계산
            marker_center_x = np.mean(corners[i][0][:, 0])
            frame_center_x = frame.shape[1] / 2
            pixel_offset = marker_center_x - frame_center_x

            x_queues[marker_id].append(x_base)
            z_queues[marker_id].append(z_distance)
            pitch_queues[marker_id].append(pitch_deg)

            now = time.time()
            if now - last_update_time > update_interval:
                last_update_time = now

                if len(pitch_queues[marker_id]) == WINDOW_SIZE:
                    avg_x = np.mean(x_queues[marker_id])
                    avg_z = np.mean(z_queues[marker_id])
                    avg_pitch = np.mean(pitch_queues[marker_id])

                    print(f"ID {marker_id}: x_base={avg_x:.3f}m, Z={avg_z:.3f}m, Pitch={avg_pitch:.1f}°, 픽셀오차={pixel_offset:.1f}px")

                    if avg_z <= Z_STOP_THRESHOLD:
                        print("도킹 완료 → 정지")
                        publish_cmd_vel(0.0, 0.0)
                        dock_enabled = False

                        print("도킹 완료 후 11cm 후진")
                        publish_cmd_vel(-0.01, 0.0)
                        time.sleep(8.5)
                        publish_cmd_vel(0.0, 0.0)
                        publish_cmd_vel(0.0, 0.0)
                        publish_cmd_vel(0.0, 0.0)
                        publish_cmd_vel(0.0, 0.0)
                        publish_cmd_vel(0.0, 0.0)


                        cap.release()
                        cv2.destroyAllWindows()
                        mqtt_client.loop_stop()
                        mqtt_client.disconnect()
                        print("second_stage.py 종료됨")
                        sys.exit(0)

                    # 픽셀 중심 기반 정렬로 후진 방향 결정
                    if avg_z > Z_THRESHOLD_1:
                        if abs(pixel_offset) < PIXEL_TOLERANCE:
                            publish_cmd_vel(-LINEAR_SPEED_1, 0.0)
                        elif pixel_offset > 0:
                            publish_cmd_vel(-LINEAR_SPEED_1, -ANGULAR_SPEED_1)
                        else:
                            publish_cmd_vel(-LINEAR_SPEED_1, ANGULAR_SPEED_1)

                    elif avg_z > Z_THRESHOLD_2:
                        if abs(pixel_offset) < PIXEL_TOLERANCE:
                            publish_cmd_vel(-LINEAR_SPEED_2, 0.0)
                        elif pixel_offset > 0:
                            publish_cmd_vel(-LINEAR_SPEED_2, -ANGULAR_SPEED_2)
                        else:
                            publish_cmd_vel(-LINEAR_SPEED_2, ANGULAR_SPEED_2)

                    else:
                        if abs(avg_pitch) < PITCH_TOLERANCE:
                            publish_cmd_vel(-LINEAR_SPEED_3, 0.0)
                        elif avg_pitch > 0:
                            publish_cmd_vel(-LINEAR_SPEED_3, -ANGULAR_SPEED_3)
                        else:
                            publish_cmd_vel(-LINEAR_SPEED_3, ANGULAR_SPEED_3)

    cv2.imshow("Docking View", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
mqtt_client.loop_stop()
mqtt_client.disconnect()
print("second_stage.py 수동 종료됨")
