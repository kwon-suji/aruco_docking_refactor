import numpy as np
import time
from docking_controller.mqtt_publisher import publish_cmd_vel

PITCH_TOLERANCE = 5.0  # degree

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
