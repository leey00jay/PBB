# PBB — ROS 2 기반 CARLA Town10 주행

PBB 자체가 ROS 2 작업공간입니다. **인지·판단·제어의 실제 알고리즘과 노드가 각 담당 패키지 안에 함께 있습니다.**
CARLA 연결 → 인지 → 판단 → 제어 → CARLA의 흐름으로 토픽을 주고받고, 화면 표시 노드가 카메라 3개와 주행 상태를 보여 줍니다.

## 실행

이 노트북에서는 새 터미널에 입력합니다.

```bash
~/Desktop/PBB/start_ros2.sh desired_speed_kmh:=20.0
```

내부에서 `ros2 launch pbb_bringup pbb.launch.py`를 실행합니다. CARLA 서버가 준비되면 노드 5개를 함께 실행합니다.
종료는 **Ctrl+C**입니다. `record:=true`는 토픽 14개를 `bags/`에 녹화합니다. 화면과 전방 원본만 저장하려면 `record:=true record_mode:=video`를 붙이면 됩니다.
속도 옵션을 주지 않으면 기존 기본값 80km/h를 사용합니다. 요청 속도이며 차선 상태·곡률·가속 제한에 따라 목표가 낮아질 수 있습니다.

ROS 명령을 직접 실행하려면:

```bash
cd ~/Desktop/PBB
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch pbb_bringup pbb.launch.py desired_speed_kmh:=20.0
```

`python_executable` 기본값은 현재 폴더의 `.venv/bin/python`입니다. 다른 위치에서는 `start_ros2.sh`를 사용하거나 `project_root`와 `python_executable`에 절대 경로를 지정합니다.

## 담당자가 수정할 곳

| 담당 | 실제 코드 | ROS 입출력 |
|---|---|---|
| **인지** | [pbb_perception](src/pbb_perception): `pbb_perception/lane_perception.py`, `lane_detector.py`, `camera_geometry.py` | 같은 폴더의 `node.py` |
| **판단** | [pbb_decision](src/pbb_decision): `pbb_decision/lane_planner.py` | 같은 폴더의 `node.py` |
| **제어** | [pbb_control](src/pbb_control): `pbb_control/vehicle_controller.py`, `pure_pursuit.py` | 같은 폴더의 `node.py` |
| CARLA | [pbb_carla](src/pbb_carla): `pbb_carla/node.py` | 차량·카메라 생성, 프레임 진행, 제어 적용 |
| 화면 | [pbb_visualization](src/pbb_visualization): `pbb_visualization/node.py` | 영상·주행 상태 구독 |
| 실행 | [pbb_bringup](src/pbb_bringup): `launch/pbb.launch.py` | 서버와 노드 실행·종료·녹화 |
| 공통 | [pbb_interfaces](src/pbb_interfaces), [pbb_common](src/pbb_common) | `.msg`, 자료형·변환·프레임 확인 |

```text
PBB/                         ROS 작업공간
├── src/
│   ├── pbb_perception/       인지 알고리즘 + 노드 + 모델 + 검사
│   ├── pbb_decision/         판단 알고리즘 + 노드 + 검사
│   ├── pbb_control/          제어 알고리즘 + 노드 + 검사
│   ├── pbb_carla/            CARLA 연결
│   ├── pbb_visualization/    화면 표시
│   ├── pbb_bringup/          launch와 서버 실행
│   ├── pbb_interfaces/       토픽 메시지
│   └── pbb_common/           공통 코드
├── tests/                   파트 통합·DDS·실행/종료 검사
├── docs/                    실행·녹화·이전 경로 안내
├── reference/aad/           교재·노트북·옛 실습 자료
├── build/ install/ log/     빌드 결과, Git 제외
└── bags/                    녹화, Git 제외
```

`src/pbb_decision/pbb_decision/`처럼 이름이 두 번 나오는 것은 바깥이 **ROS 패키지**, 안쪽이 **Python 모듈**이기 때문입니다. 안쪽 `.py`가 실제 코드입니다.
기존 `aad`를 실행 중에 찾아 불러오던 방식은 제거했습니다. 옛 위치는 `reference/aad/`에 보존했으며 실제 주행의 의존성이 아닙니다.
[파일 이동 안내](docs/ros2_migration.md) · [팀 협업 방법](CONTRIBUTING.md)

## 빌드와 수정 반영

```bash
~/Desktop/PBB/build_ros2.sh
```

처음 실행할 때 자동 빌드합니다. `--symlink-install` 방식이므로 기존 Python 파일 수정은 저장 후 주행을 재시작하면 반영됩니다.
`.msg`, `package.xml`, `setup.py`, 새 파일·패키지 추가 시에는 다시 빌드하고, 새 터미널에서 `source install/setup.bash`를 실행하세요.
`build`, `install` 안의 파일을 직접 수정하지 않습니다.

## 주행 중 확인

다른 터미널에서:

```bash
source /opt/ros/humble/setup.bash
source ~/Desktop/PBB/install/setup.bash
ros2 node list
ros2 topic list
ros2 topic echo /pbb/control/command --once
ros2 param set /pbb_decision desired_speed_kmh 20.0
```

[토픽 목록·단위·실행 옵션](docs/ros2_launch.md) · [rosbag2 녹화·공유·재생](docs/rosbag2_recording.md)

## 현재 동작과 범위

- CARLA 0.9.16 / Town10HD / Audi TT / 스폰 21. 전방 좌·중앙·우 카메라와 뒤 추적 카메라를 사용합니다.
- 중앙 카메라의 신경망 차선 인식 → 차로 중심 경로와 속도 판단 → Pure Pursuit 조향·PI 속도 제어입니다.
- 좌우 카메라는 현재 화면 확인용이며, `road_boundary_recovery` 패널은 분석 미리보기입니다. 조향 입력에 포함되지 않습니다.
- 속도는 km/h, 경로 오차는 cm입니다. 오차는 인식한 경로 기준이며 실제 도로 중심의 정답 오차가 아닙니다.
- 차선을 신뢰할 수 없거나 현재 프레임의 제어 응답이 없으면 제동합니다. 한 프레임의 제어를 적용한 뒤 다음 프레임을 진행합니다.
- 신호등·차량·보행자 대응과 교차로 경로 선택은 아직 포함되지 않습니다. 이번 재구성은 기존 판단·제어 동작을 유지하며 전체 경로 주행 성능 향상을 의미하지 않습니다.

## 다른 컴퓨터에서 준비

Ubuntu 22.04, ROS 2 Humble, Python 3.10, CARLA 0.9.16과 Town10HD가 필요합니다. 이 노트북은 NVIDIA GPU를 사용합니다.
ROS apt 저장소와 CARLA 설치는 별도입니다.

```bash
sudo apt install ros-humble-ros-base ros-humble-rosbag2-storage-default-plugins   ros-humble-rosidl-default-generators python3-colcon-common-extensions python3-pytest
git clone https://github.com/leey00jay/PBB.git
cd PBB
python3.10 -m venv .venv
source .venv/bin/activate
python -m pip install torch==2.7.1 torchvision==0.22.1 --index-url https://download.pytorch.org/whl/cu128
python -m pip install -r requirements.txt
python -m pip install ~/CARLA/PythonAPI/carla/dist/carla-0.9.16-cp310-*.whl
./build_ros2.sh
```

GPU 패키지와 `pbb_bringup/server.py`의 NVIDIA 환경 변수는 이 노트북 기준입니다.
모델은 `src/pbb_perception/models/fastai_model.pth`에 포함되고 ROS 패키지 공유 경로에 설치됩니다.

## 검사

CARLA 서버를 종료한 상태에서:

```bash
~/Desktop/PBB/test_ros2.sh
```

파트별 단위 검사, 파트 연결 회귀 검사, 독립 판단·제어 노드의 DDS 통신, 현재 launch의 시작·실패·종료 처리를 검사합니다.
파트별 검사는 각 패키지의 `test/`, 통합 검사는 루트 `tests/`에 있습니다.

## 출처와 공유 범위

- 원본: [Thomas Fermi — Algorithms for Automated Driving](https://github.com/thomasfermi/Algorithms-for-Automated-Driving), 커밋 `d13255a1ec71ed279033a1018893cfdecc26aaef`.
- [원본 라이선스](reference/aad/LICENSE) · [저작자 목록](reference/aad/CONTRIBUTORS.md) · [수정 및 자료 출처](THIRD_PARTY_NOTICES.md).
- `.venv`, `build`, `install`, `log`, `bags`, `backups`는 Git에서 제외합니다. 소스·모델·문서·검사·교재는 포함합니다.
- 녹화는 폴더 전체를 압축해 GitHub Release에 공유합니다. 코드 커밋과 녹화 업로드는 별개입니다.
