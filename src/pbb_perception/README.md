# pbb_perception

실제 알고리즘과 ROS 노드가 이 패키지에 함께 있습니다.

- 알고리즘: `pbb_perception/lane_detector.py`, `pbb_perception/lane_perception.py`, `pbb_perception/paint_tracking.py`, `pbb_perception/camera_geometry.py`.
- 중앙 카메라 신경망과 좌·중앙·우의 현재 차선 픽셀을 결합합니다. 세 영상과 차량 상태의 시각이 같은 프레임을 처리합니다.
- `paint_tracking.py`는 현재 흰색·노란색 선을 추적하며, 이전 곡선은 탐색 범위를 정하는 데 사용합니다. 실제 픽셀이 없으면 관측을 만들지 않습니다.
- `centerline.py`는 원본 3영상의 노란 이중선을 별도 검출하고 오른쪽 중앙선 여부를 메시지로 전달합니다. 가까운 측면 선은 1.5~8m 범위에서 새로 획득합니다.
- `lane_preview.py`는 실제 관측 곡선과 출처·관측 거리를 그립니다. 기존 `road_boundary_recovery.py`의 별도 미리보기는 주행 노드에서 사용하지 않습니다.
- 야간 전처리: `night_vision.py`가 원본을 보존하며 밝기를 높인 뒤 신경망에 입력합니다. 도색 검증에는 원본 중앙·좌우 영상을 사용합니다. 주간 원거리 확인: `visibility.py`가 안정된 직선의 실제 픽셀을 최대 45m까지 확인합니다.
- ROS 입출력: `pbb_perception/node.py`.
- 검사: test/.
- 토픽 형식: pbb_interfaces. Python 자료형: pbb_common.models.

PBB 루트에서:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
.venv/bin/python -m unittest discover -s src/pbb_perception/test -p 'test_*.py' -v
ros2 run --prefix "$PWD/.venv/bin/python" pbb_perception perception
```

단독 노드는 입력 토픽이 필요합니다. 전체 주행은 `./start_ros2.sh desired_speed_kmh:=20.0`입니다.
Python 수정은 저장 후 재시작, 새 파일·메시지·설정은 `./build_ros2.sh` 후 재시작합니다.
좌표·단위는 루트 docs/ros2_launch.md, 협업은 CONTRIBUTING.md를 참고하세요.

새 도색 탐색은 양쪽 기존 추적이 실패한 때만 사용합니다. 신경망·도색의 기존 곡선 추종 우선순위는 유지합니다.
