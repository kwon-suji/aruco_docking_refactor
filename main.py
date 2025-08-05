from docking_controller.first_stage import run_first_stage
from docking_controller.second_stage import run_second_stage
from docking_controller.undocking import run_undocking

def main():
    # 실행할 마커 ID와 요청 ID
    target_marker_id = 28  # 예: 마커 ID 28
    req_id = "dock_test_001"

    # 1단계 도킹 (Pitch 정렬 + 중심 맞추기)
    success = run_first_stage(target_marker_id)

    if success:
        # 2단계 도킹 (후진하여 도킹 완료)
        run_second_stage(target_marker_id)

    # 언도킹 실행
    run_undocking(target_marker_id, req_id)

if __name__ == "__main__":
    main()
