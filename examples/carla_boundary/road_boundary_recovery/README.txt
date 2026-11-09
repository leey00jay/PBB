복구 검증 강화 버전
압축을 모두 푼 뒤 run_video.bat 실행. 기존 패키지 설치했다면 setup.bat 생략.
Q/ESC로 종료. result_preview/recovery_result.mp4에서 결과를 바로 볼 수도 있습니다.
이전 버전과 구분해 새 폴더에서 사용하세요.

변경: 중앙선 색상 픽셀의 세로 범위를 더 길게 요구하고 선 지지 픽셀의 산포를 확인.
복구/급변 후보는 선 양 끝 위치를 5연속 프레임에서 확인.
선 위치 평활 및 끝점 이동 제한. 중심 오차는 평활한 선에서 다시 계산.
단순히 같은 중심값을 갖는 다른 각도의 선을 통과시키지 않도록 변경.

동일 영상 결과: tracked 146, held 40, jump_rejected 5, confirming 57, lost 120 / 총 368.
이전 버전 278프레임 -> 이번 191프레임에서 추정 출력 가능. held/jump_rejected는 새 검출이 아닌 이전 추정입니다.
연속 출력 선 끝점 최대 이동 46px -> 9px. 이는 필터 효과이고 정확도 향상 증거가 아닙니다.
문제 구간 재출력은 약 8.42초. 더 많은 NO PATH와 출력 지연이 생기는 교환관계가 있습니다.
설정한 영상과 별개 영상으로 검증이 필요합니다. 장시간 누락, 급커브, 우천/야간은 미해결.
영상상 두 경계 중간점은 m 단위 차선 중심이나 조향 명령이 아닙니다.

연동: from road_boundary_recovery import BoundaryTracker
tracker=BoundaryTracker()를 카메라 스트림마다 한 번 생성.
result=tracker.update(frame, timestamp_s), timestamp_s는 단조 증가 센서 시각(초).
판단 담당은 path_valid, detection_fresh, tracking_status, estimate_age_s를 함께 확인.
tracked: 현재 검출로 평활, held: 최대 .2초 이전 추정, jump_rejected: 급변 검출 보류,
confirming: 복구 후보 확인 중, lost: 출력 없음.
path_valid=False면 center_error_px=None. lane_valid는 도색 차선 두 개를 검출한 것이 아니므로 항상 False.
camera_ok는 입력 None 여부일 뿐. 입력 timeout/멈춘 영상 탐지는 호출자가 담당.
