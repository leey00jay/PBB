# ROS 2 중심 구조로 이동

PBB 자체가 작업공간입니다. 중첩 ros2_ws와 실행 중 aad를 찾아 불러오던 코드를 제거했습니다.
알고리즘 동작은 유지하고 위치·import·패키지 설치 방식만 바꿨습니다.

| 이전 위치 | 새 위치 |
|---|---|
| `aad/code/solutions/perception/` | `src/pbb_perception/pbb_perception/` |
| `aad/code/solutions/decision/` | `src/pbb_decision/pbb_decision/` |
| `aad/code/solutions/control/` | `src/pbb_control/pbb_control/` |
| `aad/code/solutions/interfaces.py` | `src/pbb_common/pbb_common/models.py` |
| `aad/code/solutions/lane_detection/fastai_model.pth` | `src/pbb_perception/models/fastai_model.pth` |
| `ros2_ws/src/pbb_nodes` | 각 담당 패키지의 `node.py` |
| `ros2_ws/src/pbb_interfaces` | `src/pbb_interfaces` |
| `launch/pbb.launch.py` | `src/pbb_bringup/launch/pbb.launch.py` |
| `start_carla.py` 실제 구현 | `src/pbb_bringup/pbb_bringup/server.py` (루트 파일은 바로가기) |
| `scripts/wait_for_carla.py` | `src/pbb_bringup/pbb_bringup/readiness.py` |
| 교재·이전 실습 `aad/` | `reference/aad/` |

파트별 단위 검사는 각 패키지의 test/, 전체 흐름·DDS·launch 검사는 루트 tests/에 있습니다.
옛 단일 실행과 직접 bag writer는 참고 자료로 남습니다. 새 실행은 토픽 + rosbag2 recorder를 사용합니다.

## 팀원이 pull한 뒤

이전 주행을 Ctrl+C로 종료하고 새 터미널에서:

```bash
cd ~/Desktop/PBB
./build_ros2.sh
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch pbb_bringup pbb.launch.py desired_speed_kmh:=20.0
```

이후에는 `~/Desktop/PBB/start_ros2.sh` 바로가기를 사용할 수 있습니다.
예전 `source ros2_ws/install/setup.bash`, `python -m code.tests.control...` 대신 위 명령을 사용합니다.
VS Code에서 PBB의 src를 펼치세요. 바깥 담당 폴더는 ROS 패키지, 같은 이름의 안쪽 폴더는 Python 코드입니다.
변경 전 로컬 작업은 backups/ros2_layout_날짜_시간에 보관했습니다. Git 기록·.venv·사용자 녹화는 유지합니다.
