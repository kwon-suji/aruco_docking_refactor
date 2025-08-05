import cv2
import time
import sys
import numpy as np

from docking_controller.aruco_detector import detect_markers, estimate_pose
from docking_controller.mqtt_publisher import publish_cmd_vel, stop

def run_undocking(tag_id=0, req_id="undock123"):
    UNDOCK_SPEED = 0.05
    UNDOCK_DURATION = 10
    UNDOCK_TIMEOUT = 2.0

    cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
    start_time = time.time()

    if tag_id == 0:
        print("[언도킹 시작] 마커 없이 즉시 전진")
        for _ in range(UNDOCK_DURATION):
            publish_cmd_vel(UNDOCK_SPEED, 0.0)
            time.sleep(1.0)
        publish_cmd_vel(0.0, 0.0)
        stop()
        print("[언도킹 완료]")
        return

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        corners, ids = detect_markers(frame)
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
                    stop()
                    print("[언도킹 완료]")
                    cap.release()
                    cv2.destroyAllWindows()
                    return

        if current_time - start_time > UNDOCK_TIMEOUT:
            print(f"[타임아웃] {UNDOCK_TIMEOUT}초 내 마커 ID {tag_id} 감지 실패 → 언도킹 실패")
            break

        cv2.imshow("Undocking View", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    stop()
