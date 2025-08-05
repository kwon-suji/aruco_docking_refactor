import sys
import cv2
import cv2.aruco as aruco
import numpy as np
import time
from collections import deque, defaultdict
from datetime import datetime
import json
import paho.mqtt.client as mqtt

# 전달받은 마커 ID
target_marker_id = int(sys.argv[1]) if len(sys.argv) > 1 else None
dock_enabled = True

# MQTT 설정
MQTT_BROKER = "10.10.30.62"
MQTT_PORT = 31883
CMD_VEL_TOPIC = "/ros2/cmd_vel"

mqtt_client = mqtt.Client()
mqtt_client.connect(MQTT_BROKER, MQTT_PORT)
mqtt_client.loop_start()

def publish_cmd_vel(linear=0.0, angular=0.0, req_id="auto"):
    msg = {
        "req_id": req_id,
        "linear": linear,
        "angular": angular
    }
    mqtt_client.publish(CMD_VEL_TOPIC, json.dumps(msg))

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

def move_backward_by_offset_cm(offset_cm):
    if abs(offset_cm) < 1.0:
        print("이동 생략 (1cm 미만)")
        return
    speed = -0.05
    duration = abs(offset_cm / 100.0) / abs(speed)
    print(f"→ {abs(offset_cm):.1f}cm 후진 시작")
    publish_cmd_vel(speed, 0.0, req_id="backward_move")
    time.sleep(duration)
    publish_cmd_vel(0.0, 0.0, req_id="backward_stop")
    print("→ 후진 완료")

def rotate_90(direction: str, label: str):
    angular_speed = 0.5
    duration = 3.0
    angular = angular_speed if direction == "left" else -angular_speed
    print(f"{label}: {direction.upper()} 방향으로 90도 회전")
    publish_cmd_vel(0.0, angular, req_id=f"rotate_{label}")
    time.sleep(duration)
    publish_cmd_vel(0.0, 0.0, req_id=f"stop_{label}")
    print(f"{label}: 정지")

def realign_by_offset(offset_cm):
    if abs(offset_cm) < 1.0:
        print("정렬 생략 (중심에 가까움)")
        return
    dir1 = "left" if offset_cm < 0 else "right"
    dir2 = "right" if dir1 == "left" else "left"
    rotate_90(dir1, "회전1")
    move_backward_by_offset_cm(offset_cm)
    rotate_90(dir2, "회전2")

# 카메라 캘리브레이션
camera_matrix = np.array([[486.70775596, 0., 276.18339759],
                          [0., 487.6043978, 232.99005412],
                          [0., 0., 1.]], dtype=np.float64)

dist_coeffs = np.array([[0.0512068, -0.21021301, 0.00197381, -0.02546547, 0.09921895]])
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
parameters = aruco.DetectorParameters()
marker_length = 0.093

PITCH_TOLERANCE = 5.0
ANGULAR_SPEED = 0.03
pitch_queues = defaultdict(lambda: deque(maxlen=10))

cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
last_update_time = time.time()
marker_detect_start_time = time.time()
DETECTION_TIMEOUT = 2.0  # 2초 안에 마커를 못 찾으면 실패 처리

while True:
    ret, frame = cap.read()
    if not ret:
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    # 타임아웃 검사: 해당 마커 ID가 감지되지 않음
    if ids is None or target_marker_id not in [int(i[0]) for i in ids]:
        if time.time() - marker_detect_start_time > DETECTION_TIMEOUT:
            print(f"[도킹 타임아웃] {DETECTION_TIMEOUT}초 동안 ID {target_marker_id} 마커를 찾지 못함 → 종료")

            # 로그 기록
            now_str = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            with open("docking_result.txt", "a", encoding="utf-8") as f:
                f.write(f"[{now_str}] 도킹 실패: {DETECTION_TIMEOUT}초 내 마커 ID {target_marker_id} 감지 실패\n")

            cap.release()
            cv2.destroyAllWindows()
            mqtt_client.loop_stop()
            mqtt_client.disconnect()
            sys.exit(1)
        continue

    # 타임아웃 초기화 (해당 ID 감지됨)
    marker_detect_start_time = time.time()

    rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, marker_length, camera_matrix, dist_coeffs)

    for i in range(len(ids)):
        marker_id = int(ids[i][0])
        if marker_id != target_marker_id or not dock_enabled:
            continue

        rvec, tvec = rvecs[i][0], tvecs[i][0]
        print(f"tvec[0] 원본 값: {tvec[0]:.3f} m")
        pitch_deg = rotationMatrixToEulerAngles(cv2.Rodrigues(rvec)[0])[1]
        pitch_rad = np.radians(pitch_deg)
        #X_OFFSET_BIAS = 0.15  # m (측정된 평균값)
        #x_distance = tvec[0] - X_OFFSET_BIAS
        x_distance = tvec[0]# - 0.07

        if len(pitch_queues[marker_id]) > 0:
            if abs(pitch_deg - np.mean(pitch_queues[marker_id])) > 10:
                continue

        pitch_queues[marker_id].append(pitch_deg)

        if len(pitch_queues[marker_id]) == 10 and time.time() - last_update_time > 1.0:
            last_update_time = time.time()
            avg_pitch = np.mean(pitch_queues[marker_id])
            print(f"[Pitch] ID {marker_id}: 평균 Pitch = {avg_pitch:.2f}°")

            if abs(avg_pitch) < PITCH_TOLERANCE:
                print("Pitch 정렬 완료 → 정지")
                publish_cmd_vel(0.0, 0.0)
                time.sleep(2.0)

                offset_cm = x_distance * 100
                direction_text = "왼쪽" if offset_cm > 0 else "오른쪽" if offset_cm < 0 else "중앙"
                print(f"마커 중앙으로부터 카메라 중심이 {direction_text}으로 {abs(offset_cm):.1f}cm 떨어져있음")

                now_str = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                with open("docking_result.txt", "a", encoding="utf-8") as f:
                    f.write(f"[{now_str}] 마커 ID {marker_id}, 중심선 기준 {abs(offset_cm):.1f}cm {direction_text}\n")

                realign_by_offset(offset_cm)

                cap.release()
                cv2.destroyAllWindows()
                mqtt_client.loop_stop()
                mqtt_client.disconnect()
                print("[first_stage.py] 종료됨")
                time.sleep(0.5)
                sys.exit(0)

            elif avg_pitch > PITCH_TOLERANCE:
                print("Pitch 양수 → 오른쪽 회전")
                publish_cmd_vel(0.0, -ANGULAR_SPEED)
            elif avg_pitch < -PITCH_TOLERANCE:
                print("Pitch 음수 → 왼쪽 회전")
                publish_cmd_vel(0.0, ANGULAR_SPEED)

    cv2.imshow("Pitch Alignment View", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 수동 종료 루틴
cap.release()
cv2.destroyAllWindows()
mqtt_client.loop_stop()
mqtt_client.disconnect()
print("[first_stage.py] 수동 종료됨")