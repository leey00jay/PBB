# pbb_decision

실제 알고리즘과 ROS 노드가 이 패키지에 함께 있습니다.

- 알고리즘: `pbb_decision/lane_planner.py`.
- ROS 입출력: `pbb_decision/node.py`.
- 검사: test/.
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
