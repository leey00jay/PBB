# pbb_decision

실제 알고리즘과 ROS 노드가 이 패키지에 함께 있습니다.

- 알고리즘: `pbb_decision/lane_planner.py`.
- ROS 입출력: `pbb_decision/node.py`.
- 검사: test/.
- 양쪽 실제 누락은 현재 진행 방향으로 직진하며 20km/h 목표로 감속합니다. 좌우 어느 한쪽이라도 두 번 재확인하면 중앙으로 복귀합니다. 선은 검출되나 폭·경로가 모순되는 경우의 기존 제한 보정(2초·12m)은 유지합니다.
- 오른쪽 노란 이중선 또는 관측 중앙선을 침범하는 계획은 정지하며 같은 실행 세션에서는 다시 출발하지 않습니다. 전체 실행을 종료하고 새 차량 세션으로 다시 시작해야 합니다.
- 주간은 충분히 멀리까지 보이는 안정된 직선에서 60km/h 순항을 목표로 가속하고, 새로 보이는 곡선은 미리 감속합니다. 수치·상태별 동작은 루트 `docs/ros2_launch.md`를 참고하세요.
- `pbb_common/motion.py`로 관측을 같은 좌표에 맞추며, `pbb_visualization/node.py`에서 계획을 표시합니다.
- 토픽 형식: pbb_interfaces. Python 자료형: pbb_common.models.

PBB 루트에서:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
.venv/bin/python -m unittest discover -s src/pbb_decision/test -p 'test_*.py' -v
ros2 run --prefix "$PWD/.venv/bin/python" pbb_decision decision
```

단독 노드는 입력 토픽이 필요합니다. 전체 주행은 `./start_ros2.sh desired_speed_kmh:=20.0`입니다.
Python 수정은 저장 후 재시작, 새 파일·메시지·설정은 `./build_ros2.sh` 후 재시작합니다.
좌표·단위는 루트 docs/ros2_launch.md, 협업은 CONTRIBUTING.md를 참고하세요.
