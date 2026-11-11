# 주행 화면과 전방 카메라를 rosbag2로 저장·공유하기

현재 `carla_sim_boundary.py`에 `--bag` 녹화 옵션이 있습니다.
이 노트북에는 ROS 2 Humble, rosbag2, rqt_image_view가 설치되어 있습니다.
녹화에는 추가 `pip install`이나 CARLA ROS bridge가 필요 없습니다.
영상 보기 창에서 재생하려면 아래 5단계의 압축 영상 플러그인을 처음 한 번 설치합니다.

## 무엇을 저장하나요?

| 토픽 | 내용 | 저장 방식 |
|---|---|---|
| `/pbb/screen/image/compressed` | pygame 주행 창 전체: 차선 표시, 속도, 오차, 오른쪽 분석 패널 | JPEG, 품질 85 |
| `/pbb/front/image/compressed` | 차선 표시·글자를 넣기 전의 전방 카메라 영상 | PNG, 무손실 |

두 토픽의 메시지 형식은 `sensor_msgs/msg/CompressedImage`입니다.
`compressed`는 영상 프레임을 압축했다는 뜻이며, 전방 PNG는 복원 시 원본 RGB 픽셀과 같습니다.
바탕화면 전체가 아닌 pygame 창의 내용만 기록합니다.
`--half_image` 사용 시 전방 원본은 512×256, 주행 화면은 1200×600입니다.
차량 제어값이나 위치를 별도 수치 토픽으로 저장하지 않으며, 속도·오차는 화면에 표시된 모습으로 남습니다.

## 1. CARLA 서버 실행

이미 서버가 켜져 있다면 이 단계는 건너뜁니다.

```bash
cd ~/Desktop/PBB
python3 start_carla.py
```

기존 주행 프로그램은 해당 터미널에서 `Ctrl+C`로 종료하세요. CARLA 서버는 켜 둡니다.
두 주행 프로그램이 동시에 같은 세계를 제어하지 않도록 합니다.

## 2. 새 터미널에서 녹화하며 주행

```bash
source /opt/ros/humble/setup.bash
cd ~/Desktop/PBB
source .venv/bin/activate
cd aad
python -m code.tests.control.carla_sim_boundary --ld --half_image --bag ../bags/town10_01
```

- `source /opt/ros/humble/setup.bash`: 이 터미널에서 ROS 2 기능을 사용할 준비.
- `--bag ../bags/town10_01`: PBB의 `bags/town10_01` 폴더에 기록.
- `--ld`: 전방 인식 카메라를 생성하는 모드이므로 녹화할 때 필요합니다.
- 창 오른쪽 아래 `REC ROSBAG`와 프레임 수가 증가하면 저장 중입니다.

이 프로그램은 Python API로 bag을 직접 쓰므로 별도 `ros2 bag record` 명령은 필요 없습니다.
녹화 중 ROS 토픽을 실시간 발행하지 않습니다. 저장 후 `ros2 bag play`를 실행하면 발행됩니다.
프레임을 계속 메모리에 쌓지 않고 파일로 기록합니다. 압축·디스크 쓰기로 실제 실행 속도가 낮아질 수 있습니다.
우선 10~20초 정도 시험 녹화한 뒤 확인하는 것이 좋습니다.

`town10_01` 폴더가 이미 있으면 덮어쓰지 않고 중단합니다.
다음 촬영에는 `town10_02`처럼 다른 이름을 지정하세요.
폴더 이름 없이 `--bag`만 쓰면 `PBB/bags/town10_날짜_시간` 형식으로 자동 생성합니다.

## 3. 녹화 종료와 확인

주행 명령을 실행한 터미널에서 **Ctrl+C를 한 번** 누르고 `ROS bag saved`가 뜰 때까지 기다립니다.
이때 `metadata.yaml`을 포함한 bag 파일 마무리가 이루어집니다.
프로세스를 강제 종료하거나 녹화 중 폴더를 압축하면 완성되지 않은 자료가 될 수 있습니다.

```bash
ros2 bag info ~/Desktop/PBB/bags/town10_01
```

두 토픽이 모두 보이고 각각 `Count`가 0보다 큰지 확인하세요. 정상 종료 시 두 개의 수가 같습니다.

```text
PBB/bags/town10_01/
├── metadata.yaml
├── town10_01_0.db3
└── recording_info.json
```

`recording_info.json`에는 맵, 차량, 카메라 설정, 저장한 프레임 수, 시간 기준이 들어갑니다.
영상 시각은 첫 프레임을 컴퓨터 시각에 맞추고 이후에는 CARLA 시뮬레이션 시간 간격을 보존합니다.
따라서 느린 컴퓨터에서 실제로 1분을 실행해도 녹화 분량은 시뮬레이션 시간 기준으로 더 짧을 수 있습니다.

## 4. 압축해서 공유

녹화가 끝난 다음 실행합니다.

```bash
cd ~/Desktop/PBB/bags
zip -r town10_01.zip town10_01
```

완성된 `~/Desktop/PBB/bags/town10_01.zip`을 카카오톡의 파일 첨부나 팀에서 쓰는 드라이브로 전달하세요.
`.db3` 하나만 보내지 말고 **폴더 전체**를 공유합니다.
압축 후 원본 폴더도 그대로 남습니다. 이미 만든 ZIP을 갱신하기보다는 촬영마다 다른 이름을 사용하세요.
bag 및 이 폴더 안의 ZIP은 용량이 커질 수 있어 Git에 포함되지 않습니다.
받는 사람에게 이 문서도 함께 전달하면 재생할 수 있습니다.

## 5. 받은 자료 재생

ROS 2 Humble의 rosbag2와 rqt_image_view, compressed image transport가 설치된 컴퓨터에서 사용합니다.
이 노트북에는 **압축 영상 플러그인만 빠져 있으므로 처음 한 번** 설치합니다.

```bash
sudo apt install ros-humble-compressed-image-transport
```

노트북 로그인 암호를 입력하고 Enter를 누릅니다. 암호를 칠 때 글자가 표시되지 않아도 정상입니다.
관리자 암호가 필요해 자동 설치는 완료하지 않았습니다. 이 설치는 저장한 영상을 rqt에서 볼 때 필요하며 녹화에는 필요 없습니다.
다른 Ubuntu 22.04 / ROS 2 Humble 컴퓨터에서는 `ros-humble-rosbag2`, `ros-humble-rqt-image-view`도 필요합니다.

받는 사람은 CARLA 서버나 신경망 모델을 켤 필요가 없습니다.
먼저 ZIP을 풀어 `metadata.yaml`이 들어 있는 폴더를 찾습니다.

**터미널 A — 반복 재생** (이 노트북의 경로 예시)

```bash
source /opt/ros/humble/setup.bash
ros2 bag play ~/Desktop/PBB/bags/town10_01 --loop
```

**터미널 B — 영상 보기**

```bash
source /opt/ros/humble/setup.bash
ros2 run rqt_image_view rqt_image_view
```

영상 창의 목록에서 `/pbb/screen/image/compressed`를 선택하면 주행 화면,
`/pbb/front/image/compressed`를 선택하면 전방 원본입니다.
목록 표시는 버전에 따라 `…/image (compressed)` 형태일 수도 있습니다.
목록이 비어 있으면 재생이 실행 중인지 확인한 뒤 새로고침합니다.
두 영상을 동시에 보려면 영상 보기 명령을 새 터미널에서 한 번 더 실행합니다.
종료할 때는 재생 터미널에서 `Ctrl+C`를 누릅니다.

`ros2 bag play`는 저장된 메시지를 다시 보내는 기능이므로 그 명령만으로 영상 창이 뜨지는 않습니다.
이 bag에는 영상만 들어 있어 CARLA 세계를 복원하거나 차를 다시 움직이지 않습니다.
전방 영상 토픽을 구독하면 같은 프레임으로 다른 인식 코드를 시험할 수 있습니다.

## 구현과 검증

- 저장 구현: `aad/code/solutions/runtime/rosbag_recorder.py`
- 화면 연결: `aad/code/tests/control/carla_sim_boundary.py`
- 테스트: `aad/code/tests/runtime/test_rosbag_recorder.py`
- 녹화 옵션을 사용하지 않을 때는 ROS 모듈을 불러오지 않습니다.
- 합성 영상으로 실제 ROS bag 쓰기·읽기, 두 토픽 시각 일치, PNG 원본 일치, 정상·예외 종료 후 메타데이터 생성을 검사합니다. `ros2 bag play`가 보낸 두 영상 수신과 ROS `cv_bridge`로 원본 복원도 확인했습니다. rqt 영상 창과 실제 Town10 주행 녹화는 별도 확인이 필요합니다.

공식 참고: [ROS 2 Python API로 bag 직접 저장하기](https://github.com/ros2/ros2_documentation/blob/humble/source/Tutorials/Advanced/Recording-A-Bag-From-Your-Own-Node-Py.rst),
[rosbag2 재생·조회 사용법](https://github.com/ros2/rosbag2/tree/humble).
