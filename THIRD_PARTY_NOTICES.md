# 출처와 수정 내역

기초 알고리즘과 모델: Thomas Fermi 및 기여자의 Algorithms for Automated Driving.
https://github.com/thomasfermi/Algorithms-for-Automated-Driving
원본 커밋: d13255a1ec71ed279033a1018893cfdecc26aaef
라이선스: CC BY 4.0, https://creativecommons.org/licenses/by/4.0/
원본 라이선스와 저작자: reference/aad/LICENSE, reference/aad/CONTRIBUTORS.md.

PBB 수정: Town10HD 설정, km/h·소수점 오차 표시, 차선 누락 처리, 곡률·목표 속도 제한,
Pure Pursuit 입력 보존, PI 속도 제어, PyTorch 호환, 카메라 3개 표시,
ROS 메시지·노드·launch·녹화·검사. 실제 알고리즘 구현은 src/로 이동했습니다.

src/pbb_perception/pbb_perception/road_boundary_recovery.py는 사용자가 제공한
road_boundary_recovery_windows.zip의 코드입니다. AAD 원본에서 가져온 파일이 아니며,
이번 구조 변경으로 해당 코드에 별도의 라이선스를 부여하지 않습니다.

map.osm, my_map.xodr는 OpenStreetMap 자료 및 변환물입니다.
© OpenStreetMap contributors, https://www.openstreetmap.org/copyright
