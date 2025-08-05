import cv2
import cv2.aruco as aruco
import numpy as np

# 카메라 캘리브레이션 매트릭스 (사용자 값으로 교체 가능)
camera_matrix = np.array([
    [486.70775596, 0., 276.18339759],
    [0., 487.6043978 ,232.99005412],
    [0., 0., 1.]
], dtype=np.float64)

dist_coeffs = np.array([[0.0512068, -0.21021301, 0.00197381, -0.02546547, 0.09921895]])

# ArUco 설정
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
parameters = aruco.DetectorParameters()
marker_length = 0.093  # 단위: meter

def detect_markers(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    return corners, ids

def estimate_pose(corners):
    rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
        corners, marker_length, camera_matrix, dist_coeffs
    )
    return rvecs, tvecs

def draw_markers(frame, corners, ids):
    return aruco.drawDetectedMarkers(frame, corners, ids)
