# PBB — ROS 2 기반 CARLA Town10 주행

PBB 자체가 ROS 2 작업공간입니다. **인지·판단·제어의 실제 알고리즘과 노드가 각 담당 패키지 안에 함께 있습니다.**
CARLA 연결 → 인지 → 판단 → 제어 → CARLA의 흐름으로 토픽을 주고받고, 화면 표시 노드가 카메라 3개와 주행 상태를 보여 줍니다.

## 실행

이 노트북에서는 새 터미널에 입력합니다.

```bash
~/Desktop/PBB/start_ros2.sh desired_speed_kmh:=60.0
```

내부에서 `ros2 launch pbb_bringup pbb.launch.py`를 실행합니다. CARLA 서버가 준비되면 노드 5개를 함께 실행합니다.
종료는 **Ctrl+C**입니다. `record:=true`는 토픽 14개를 `bags/`에 녹화합니다. 화면과 전방 원본만 저장하려면 `record:=true record_mode:=video`를 붙이면 됩니다.
속도 옵션을 주지 않으면 기본값 60km/h를 사용합니다. 요청 속도이며 차선 상태·곡률·가속 제한에 따라 목표가 낮아질 수 있습니다.

주간 우천 환경을 선택하려면:

```bash
~/Desktop/PBB/start_ros2.sh weather:=MidRainyNoon
```

`SoftRainNoon`은 약한 비, `MidRainyNoon`은 보통 비, `HardRainNoon`은 강한 비입니다.
옵션을 생략하면 `ClearNoon`(맑은 주간)입니다. 비·물웅덩이·노면 젖음을 적용하며 화면 오른쪽에 환경 이름이 표시됩니다.
자세한 값과 녹화 명령은 [주간 우천 안내](docs/ros2_launch.md#주간-우천-환경)를 참고하세요.

맑은 야간은 아래 명령으로 실행합니다. 비 오는 야간은 `MidRainyNight`로 바꾸세요.

```bash
~/Desktop/PBB/start_ros2.sh weather:=ClearNight
```

야간에는 차량의 차폭등과 하향등을 켭니다. `SoftRainNight`(약한 비), `HardRainNight`(강한 비)도 선택할 수 있습니다.
[야간 설정 안내](docs/ros2_launch.md#야간-환경)

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
- 야간 프리셋은 중앙 영상의 밝기를 감마 0.65로 보정한 뒤 신경망으로 인식합니다. 도색 검증은 원본 3개 영상을 사용해 밝기 보정으로 강조된 반사광이 경계로 확정되지 않도록 합니다. 하단 카메라·원본 토픽은 원본이고, 상단 인식 화면은 보정 영상입니다. `night_gamma`를 낮추면 더 밝아지며 `night_enhancement:=false`로 끌 수 있습니다. 주간 영상은 보정하지 않습니다.
- 중앙 카메라 신경망과 전방 3개 카메라의 현재 흰색·노란색 차선 픽셀을 결합해 경로를 추정합니다. 좌우 영상도 인식에 사용합니다.
- 짧은 점선은 기존 곡률을 유지해 보정하고, 충분한 실제 픽셀이 보이면 곡률을 새로 맞춥니다. 오른쪽 패널은 별도 도로 미리보기 대신 실제 관측한 차선 곡선과 검출 출처·관측 거리를 보여 줍니다.
- 양쪽 차선이 없으면 `NO LANES - STRAIGHT`: 인식을 잃은 순간의 방향으로 직진하며 목표 20km/h로 감속합니다. 예전 2초·12m 제한으로 자동 정지하지 않습니다. 차선이 돌아와 좌우 중 한쪽이라도 일치하는 관측 2회가 짧은 시간 안에 확인되면 중앙 추적으로 복귀합니다. 선이 검출되지만 폭·경로가 모순되는 경우는 기존의 최대 2초·12m 보정 후 정지를 유지합니다. 양쪽 실제 소실과 구분합니다.
- 직선에서는 18m 이상 실제 차선이 보이고 중앙·방향이 0.7초 이상 안정되면 가속합니다. 주간 직선 목표는 양쪽·한쪽 모두 60km/h이며, 사용자 요청과 실제 관측 거리로 계산한 감속 여유가 추가 제한합니다. 주간에는 현재 영상의 먼 직선 차선을 최대 45m까지 확인하며, 빈 구간을 넘어 임의로 관측 거리를 늘리지 않습니다. 야간 한쪽 상한은 기존 55km/h입니다. 단편적인 관측은 35~40km/h, 곡선은 곡률에 따라 28~35km/h 이하로 줄입니다.
- 곡선에서는 Pure Pursuit의 기준점을 가까이 잡아 차로 중앙을 추종합니다. 초록색은 계획 중심, 노란 점선은 가상 경계입니다. `path error`는 관측으로 만든 계획에 대한 오차이며 지도 정답 오차가 아닙니다. 무차선 직진 중에는 오차를 `--`로 표시합니다.
- 잘못된 차량 상태·제어 응답 시간 초과·실행 초기 준비·중앙선 침범에는 제동이 유지됩니다. 양쪽 무인식 시 직진하는 기능은 도로 진행 방향이나 장애물을 확인하는 기능이 아닙니다.
- 이전 008 변경의 ClearNoon 1,075프레임 검증: 최고 실제 55.60km/h, 준비 구간 제외 평균 29.36km/h입니다. 첫 곡선 중심 오차는 평균 0.137m·최대 0.322m였으나 이후 교차로에서 반대 차로로 진입해 검증을 중단했습니다. 직선 60km/h 안팎 순항이 목표이며 전체 평균은 목표 기준이 아닙니다. 당시 전체 경로 완주는 달성하지 못했습니다.
- 이전 008 변경의 ClearNight 1,000프레임에서는 첫 곡선을 같은 차로로 통과했고 중심 오차는 평균 0.097m·최대 0.260m였습니다. MidRainyNight는 원본과 보정 입력 모두 반사로 인한 폭 불일치가 남았습니다. 밝기 보정만으로 야간 우천이 해결된 것은 아닙니다. 신호등·차량·보행자 대응과 교차로 방향 선택도 포함되지 않습니다.

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

노란 이중선은 별도 중앙선으로 추적합니다. 오른쪽 이중선이나 중앙선을 침범하는 계획이 검출되면 목표 0·브레이크 1로 정지하고 해당 실행 세션 동안 유지합니다. 중앙선 검출 자체의 완전성을 보장하는 기능은 아닙니다. `LaneEstimate` 형식이 바뀌었으므로 `./build_ros2.sh` 후 전체 실행을 다시 시작하세요. 상세 동작은 [실행 안내](docs/ros2_launch.md)를 참고하세요.

009 최종 ClearNoon 1,200프레임에서는 비교차로의 반대 방향 프레임 0, 직진 탐색 후 한쪽 차선 복귀와 중앙선 위험 제동을 확인했습니다. 다만 첫 곡선에서 같은 방향의 옆 차로로 밀린 구간이 있으며 중심 오차는 최대 1.752m입니다. 중앙 유지가 완전히 해결된 결과는 아닙니다.
