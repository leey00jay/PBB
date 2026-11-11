# 인지·판단·제어 협업

PBB는 ROS 2 작업공간이며 실제 주행 코드는 src/에 있습니다.

| 담당 | 알고리즘 수정 위치 | 통신 | 검사 |
|---|---|---|---|
| 인지 | `src/pbb_perception/pbb_perception/` | `node.py`: 영상+상태 → 차선 | `src/pbb_perception/test/` |
| 판단 | `src/pbb_decision/pbb_decision/` | `node.py`: 차선 → 경로+속도 | `src/pbb_decision/test/` |
| 제어 | `src/pbb_control/pbb_control/` | `node.py`: 경로 → 조향+액셀+브레이크 | `src/pbb_control/test/` |

`reference/aad`는 교재·옛 실습 자료입니다. 여기의 호환 import 파일 대신 src의 실제 알고리즘을 수정합니다.
토픽 필드는 `src/pbb_interfaces/msg`, Python 자료형과 변환은 `src/pbb_common/pbb_common/models.py`, `wire.py`에 있습니다.
좌표·단위·프레임 계약은 [ROS 안내](docs/ros2_launch.md)를 따릅니다.

## 작업 시작과 확인

진행 중인 변경사항을 확인하고 보관한 뒤 최신 main에서 작업합니다. 인지 담당자의 예시입니다.

```bash
cd ~/Desktop/PBB
git status
git switch main
git pull --ff-only
git switch -c codex/perception-work
./build_ros2.sh
```

이미 작업 브랜치가 있으면 이어서 사용합니다. 구조 변경을 받은 뒤에는 새 터미널에서 `source install/setup.bash`를 사용합니다.
Python 파일 수정은 저장 후 재시작합니다. 새 파일·메시지·패키지 설정은 다시 빌드합니다.

```bash
./test_ros2.sh
./start_ros2.sh desired_speed_kmh:=20.0
git diff
git status
```

CARLA를 종료한 상태에서 검사합니다. 인식 성능은 주행 조건과 녹화로 별도 확인합니다.
수정한 파일을 선택해서 커밋하고 작업 브랜치를 올린 뒤 main에 대한 PR을 만듭니다.
PR에는 바뀐 동작, 확인한 토픽·영상·주행 조건, 검사 결과를 적습니다.
`build`, `install`, `.venv`, `bags`, `backups`, 로컬의 중첩 PBB 체크아웃은 커밋 대상이 아닙니다.
