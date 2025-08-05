import cv2
import time
import numpy as np
from collections import deque, defaultdict

from docking_controller.aruco_detector import detect_markers, estimate_pose
from docking_controller.alignment_controller import rotationMatrixToEulerAngles
from docking_controller.mqtt_publisher import publish_cmd_vel, stop

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

CAMERA_OFFSET = 0.45  # meters
Z_THRESHOLD_1 = 0.5
Z_THRESHOLD_2 = 0.25
Z_STOP_THRESHOLD = 0.2

def run_second_stage(target_marker_id):
    cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
    last_update_time = time.time()
    update_interval = 1.0
    dock_enabled = True

    while True:
        ret, frame = cap.read()
        if not ret or frame is None:
            break

        corners, ids = detect_markers(frame)

        if ids is not None:
            rvecs, tvecs = estimate_pose(corners)

            for i in range(len(ids)):
                marker_id = int(ids[i][0])
                if not dock_enabled or marker_id != target_marker_id:
                    continue

                rvec = rvecs[i][0]
                tvec = tvecs[i][0]

                x_cam = tvec[0]
                z_cam = tvec[2]
                pitch_deg = rotationMatrixToEulerAngles(cv2.Rodrigues(rvec)[0])[1]
                pitch_rad = np.radians(pitch_deg)

                x_base = x_cam + CAMERA_OFFSET * np.sin(pitch_rad)
                z_distance = z_cam

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

                        print(f"ID {marker_id}: x={avg_x:.3f}, z={avg_z:.3f}, pitch={avg_pitch:.2f}, pixel offset={pixel_offset:.1f}")

                        if avg_z <= Z_STOP_THRESHOLD:
                            print("도킹 완료 → 정지")
                            publish_cmd_vel(0.0, 0.0)
                            dock_enabled = False

                            print("도킹 완료 후 11cm 후진")
                            publish_cmd_vel(-0.01, 0.0)
                            time.sleep(8.5)
                            publish_cmd_vel(0.0, 0.0)
                            stop()
                            cap.release()
                            cv2.destroyAllWindows()
                            print("[second_stage.py] 종료됨")
                            return

                        # z 거리 따라 후진 제어
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
    stop()
    print("[second_stage.py] 수동 종료됨")
