import cv2
import time
import sys
import numpy as np
from collections import deque, defaultdict
from datetime import datetime

from docking_controller.aruco_detector import detect_markers, estimate_pose
from docking_controller.alignment_controller import rotationMatrixToEulerAngles, realign_by_offset
from docking_controller.mqtt_publisher import publish_cmd_vel, stop

PITCH_TOLERANCE = 5.0
ANGULAR_SPEED = 0.03
pitch_queues = defaultdict(lambda: deque(maxlen=10))

def run_first_stage(target_marker_id):
    cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
    last_update_time = time.time()
    marker_detect_start_time = time.time()
    DETECTION_TIMEOUT = 2.0

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        corners, ids = detect_markers(frame)

        if ids is None or target_marker_id not in [int(i[0]) for i in ids]:
            if time.time() - marker_detect_start_time > DETECTION_TIMEOUT:
                print(f"[도킹 타임아웃] {DETECTION_TIMEOUT}초 동안 ID {target_marker_id} 마커 감지 실패 → 종료")

                now_str = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                with open("docking_result.txt", "a", encoding="utf-8") as f:
                    f.write(f"[{now_str}] 도킹 실패: 마커 ID {target_marker_id} 감지 실패\n")

                cap.release()
                cv2.destroyAllWindows()
                stop()
                return False
            continue

        marker_detect_start_time = time.time()
        rvecs, tvecs = estimate_pose(corners)

        for i in range(len(ids)):
            marker_id = int(ids[i][0])
            if marker_id != target_marker_id:
                continue

            rvec = rvecs[i][0]
            tvec = tvecs[i][0]
            print(f"tvec[0] 원본 값: {tvec[0]:.3f} m")

            pitch_deg = rotationMatrixToEulerAngles(cv2.Rodrigues(rvec)[0])[1]
            pitch_rad = np.radians(pitch_deg)
            x_distance = tvec[0]  # 필요시 보정값 추가

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
                    print(f"마커 중앙에서 {direction_text}으로 {abs(offset_cm):.1f}cm 떨어져 있음")

                    now_str = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                    with open("docking_result.txt", "a", encoding="utf-8") as f:
                        f.write(f"[{now_str}] 마커 ID {marker_id}, {abs(offset_cm):.1f}cm {direction_text}\n")

                    realign_by_offset(offset_cm)

                    cap.release()
                    cv2.destroyAllWindows()
                    stop()
                    print("[first_stage.py] 종료됨")
                    time.sleep(0.5)
                    return True

                elif avg_pitch > PITCH_TOLERANCE:
                    print("Pitch 양수 → 오른쪽 회전")
                    publish_cmd_vel(0.0, -ANGULAR_SPEED)
                elif avg_pitch < -PITCH_TOLERANCE:
                    print("Pitch 음수 → 왼쪽 회전")
                    publish_cmd_vel(0.0, ANGULAR_SPEED)

        cv2.imshow("Pitch Alignment View", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    stop()
    print("[first_stage.py] 수동 종료됨")
    return False
