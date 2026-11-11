# PBB — CARLA Town10 실습

`pbb2`에서 직접 수정하고 실행한 CARLA 실습을 PBB로 옮긴 프로젝트입니다.
2026-11-11 기준의 코드를 보관하며, 기존 PBB의 파일 구성은 이 구성으로 교체했습니다.

## 파트별 작업 폴더

| 파트 | 바로가기 | 담당 내용 |
|---|---|---|
| **인지** | [perception](aad/code/solutions/perception) | 영상 인식, 경계 곡선 추출, 카메라 좌표 계산 |
| **판단** | [decision](aad/code/solutions/decision) | 중심 경로 생성, 목표 속도와 주행·정지 결정 |
| **제어** | [control](aad/code/solutions/control) | 조향·액셀·브레이크 계산 |
| 연결 | [runtime](aad/code/solutions/runtime) | 세 파트를 순서대로 호출하고 기존 CARLA 화면과 연결 |

각 폴더의 README에 담당 파일·입출력·다음 작업·검사 명령이 있습니다.
[공통 전달 형식](aad/code/solutions/interfaces.py) · [팀 작업 방법](CONTRIBUTING.md)

```text
aad/code/solutions/
├── perception/       인지
├── decision/         판단
├── control/          제어
├── runtime/          세 파트 연결
├── interfaces.py     공통 데이터 형식
└── lane_detection/   이전 import 호환 파일과 기존 모델
```

**인지·판단·제어를 파트별로 분리했고, 카메라 모드의 속도 제어를 PI 방식으로 개선했습니다.**
새 도로 경계 검출기를 차량 주행에 연결하는 작업은 아직 남아 있습니다.

## 현재 구현 상태

- CARLA 0.9.16 / Town10HD / Audi TT / 스폰 번호 21.
- 전방 카메라 영상의 차선을 기존 신경망으로 검출하고, 추정한 중심 경로를 Pure Pursuit로 따라갑니다.
- 두 실행 파일의 요청 속도(`desired_speed_kmh`)는 **80km/h**입니다. 판단 파트는 한쪽 차선만 보이면 최대 40km/h로 제한하고, 전방 곡률에 따라 더 낮춥니다. 최종 목표 속도에는 가속·감속 변화량 제한도 적용됩니다. 80km/h 차선 유지 성능을 검증한 것은 아닙니다.
- 액셀 상한은 0.75이고, PI 제어로 목표 속도에 부족한 출력을 보정합니다. Town10 Audi TT 직선 시험에서 20km/h 목표에 약 19.99km/h로 수렴했습니다. 경로를 신뢰하지 못하면 정지하며, Town10 전체 구간 주행 성능을 검증한 것은 아닙니다.
- `road_boundary_recovery.py`는 노란 중앙선과 오른쪽 도로 가장자리 후보를 찾습니다. `carla_sim_boundary.py`의 오른쪽 패널에 실시간 결과를 표시합니다.
- **새 도로 경계 검출 결과는 아직 차량 조향에 사용하지 않습니다.** 차량은 기존 차선 인식 결과로 움직입니다. 오른쪽의 `tracked`와 왼쪽의 `STOP`은 동시에 나타날 수 있습니다.
- 신호등·보행자·차량 대응이나 교차로 경로 선택까지 완성한 자율주행 시스템은 아닙니다.

## 이 노트북에서 실행

VS Code에서 `~/Desktop/PBB` 폴더를 여세요. 터미널은 두 개를 사용합니다.
이 노트북에 이전해 둔 `.venv`가 있다면 패키지를 다시 설치할 필요가 없습니다.

**터미널 1 — CARLA 서버**

```bash
cd ~/Desktop/PBB
python3 start_carla.py
```

서버 창이 열린 다음, **터미널 2 — 주행 및 도로 경계 분석 화면**:

```bash
cd ~/Desktop/PBB
source .venv/bin/activate
cd aad
python -m code.tests.control.carla_sim_boundary --ld --half_image
```

- 왼쪽: 기존 차선 인식과 차량 주행 상태.
- 오른쪽: 새 도로 경계 검출 결과. `Used for steering: NO`는 조향에 미연결이라는 뜻입니다.
- 주행 프로그램을 종료하려면 그 터미널에서 `Ctrl+C`를 누릅니다.
- 기존 주행 프로그램이 실행 중이라면 종료한 다음 새 프로그램을 실행하세요. 두 프로그램이 같은 CARLA 세계를 동시에 제어하면 안 됩니다.
- 코드 수정 후 `Ctrl+S`로 저장하고 주행 프로그램을 다시 실행해야 반영됩니다.

오른쪽 분석 패널 없이 기존 차선 유지 화면만 실행하려면 마지막 명령을 바꾸세요.

```bash
python -m code.tests.control.carla_sim --ld --half_image
```

`--ld`는 카메라 기반 차선 인식 모드입니다. 이 옵션을 빼면 지도 기반 경로 추종 모드로 실행됩니다.
`--half_image`는 전방 인식 카메라 해상도를 512×256으로 낮춥니다.
현재 주행 화면용 카메라와 전방 인식용 카메라를 사용하며, 전방 카메라 3개 구성은 아직 적용하지 않았습니다.

## 주행 화면과 전방 원본을 rosbag2로 저장

ROS 2 Humble 환경을 불러온 뒤 `carla_sim_boundary` 주행 명령에 `--bag ../bags/town10_01`을 붙이면
주행 화면(JPEG)과 차선 표시 없는 전방 원본(PNG)을 함께 저장합니다.
`Ctrl+C`로 종료하면 bag 폴더를 완성하며, `/bags/`는 Git에서 제외됩니다.
설치 확인, 실행, 압축 공유, 재생 순서는 [rosbag2 녹화 안내](docs/rosbag2_recording.md)를 보세요.

## 파일 위치

| 파일 | 역할 |
|---|---|
| `start_carla.py` | `~/CARLA`의 서버 실행. 이 노트북의 NVIDIA 설정과 800×600 창 크기 사용 |
| `connect_carla.py` | 서버·API 버전과 현재 맵 확인, 맑은 낮 날씨 설정 |
| `spawn_car.py` | 차량 생성과 관찰 시점 이동 실습 |
| `aad/code/tests/control/carla_sim_boundary.py` | Town10 주행 + 새 도로 경계 분석 패널 |
| `aad/code/tests/control/carla_sim.py` | 기존 차선 유지 주행 화면 |
| `aad/code/solutions/perception/lane_perception.py` | 검출 결과에서 유효한 차선 경계 추출 |
| `aad/code/solutions/perception/lane_detector.py` | 기존 신경망 모델로 차선 검출 |
| `aad/code/solutions/decision/lane_planner.py` | 경로 생성, 목표 속도·주행·정지 판단 |
| `aad/code/solutions/control/vehicle_controller.py` | 현재 카메라 모드의 액셀·브레이크·조향 제한 |
| `aad/code/solutions/runtime/lane_following.py` | 인지 → 판단 → 제어 연결 |
| `aad/code/solutions/lane_detection/fastai_model.pth` | 기존 신경망 모델. 저장소에 포함 |
| `aad/code/solutions/perception/road_boundary_recovery.py` | 전달받은 새 도로 경계 검출 코드 |
| `aad/code/solutions/control/pure_pursuit.py` | 경로를 따라갈 조향각 계산 |
| `aad/code/tests/control/test_lane_following.py` | 차선 누락과 제어 오류 처리 검사. 실제 주행 설정 파일이 아님 |
| `map.osm`, `my_map.xodr` | 이전에 만든 지도 실습 자료. Town10 실행에는 사용하지 않음 |
| `aad/book`, `aad/data`, `aad/code/exercises` | 원본 오픈소스의 교재·자료·연습 코드 |

주행 화면의 `speed (km/h)`는 실제 속도, `target (km/h)`는 현재 제어 목표입니다.
`throttle / brake`는 현재 명령의 액셀·브레이크 값(각각 0~1)입니다.
`lateral error`와 `max lat. error`는 **카메라로 추정한 경로 기준**의 오차이며, 실제 도로 중심에 대한 정답 오차는 아닙니다.
오른쪽 패널의 중심 오차는 화면 픽셀 단위로, cm나 m 단위 오차와 다릅니다.

## 다른 컴퓨터에서 준비

검증에 사용한 환경은 Ubuntu, Python 3.10, CARLA 0.9.16, NVIDIA GPU입니다.
CARLA 서버와 Town10HD 맵은 별도로 설치해야 합니다. CARLA 설치 경로는 기본적으로 `~/CARLA`입니다.
`start_carla.py`의 NVIDIA 관련 환경 변수는 해당 노트북용이므로 다른 그래픽 환경에서는 조정이 필요합니다.

```bash
git clone https://github.com/leey00jay/PBB.git
cd PBB
python3.10 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install torch==2.7.1 torchvision==0.22.1 --index-url https://download.pytorch.org/whl/cu128
python -m pip install -r requirements.txt
python -m pip install ~/CARLA/PythonAPI/carla/dist/carla-0.9.16-cp310-*.whl
```

CARLA 설치본의 Python API wheel 파일명이 다른 경우에는 실제 파일 경로를 지정하세요.
위 설치 명령은 이 노트북의 주행 환경을 기준으로 작성했습니다. 다른 컴퓨터에서 전체 재설치까지 검증한 것은 아닙니다.
원본의 `aad/code/environment.yml`은 오래된 교재 환경이며, 현재 주행 실습은 루트의 `requirements.txt`를 기준으로 합니다.
교재의 노트북·학습·녹화 기능에는 별도 패키지가 필요할 수 있습니다. 예를 들어 `--vid`에는 `imageio`, `imageio-ffmpeg`, `tqdm`이 추가로 필요합니다.

## 검사

CARLA를 켜지 않고 실행할 수 있습니다.

```bash
cd ~/Desktop/PBB
source .venv/bin/activate
cd aad
python -m unittest discover -s code/tests -t . -p 'test_*.py' -v
```

이 검사는 제어 코드의 일부 동작을 확인합니다. 실제 Town10 전체 경로 주행 성능을 검증하는 테스트는 아닙니다.

## GitHub에 포함하는 범위와 출처

실습 코드, 신경망 모델, 교재와 예제 데이터, 지도 자료를 포함합니다.
`.venv`, `__pycache__`, `backups`, 생성된 실행 로그·녹화 파일은 로컬에 두고 Git에서 제외합니다.
`aad`는 이 저장소에 포함된 일반 폴더이며 Git 서브모듈이 아닙니다.

- 원본: [Thomas Fermi — Algorithms for Automated Driving](https://github.com/thomasfermi/Algorithms-for-Automated-Driving)
- 가져온 원본 커밋: `d13255a1ec71ed279033a1018893cfdecc26aaef`
- 원본 저작자와 라이선스 안내: [aad/CONTRIBUTORS.md](aad/CONTRIBUTORS.md), [aad/LICENSE](aad/LICENSE). 원본은 CC BY 4.0으로 제공됩니다.
- 주요 수정: Town10HD 설정, km/h 표시, 오차 소수점 표시, 차선 누락 시 경로 추정과 정지 처리, Pure Pursuit의 입력 경로 변경 방지, PyTorch 모델 로드 호환 처리, 새 경계 분석 패널 연결, 목표 속도 제한 변경.
- `perception/road_boundary_recovery.py`는 사용자가 제공한 `road_boundary_recovery_windows.zip`에서 가져왔으며, 해당 검출기 파일 자체는 전달받은 내용과 같습니다. 위 원본 AAD 저장소에서 가져온 파일은 아닙니다. 이전 `lane_detection` 경로에는 호환 import를 남겼습니다.
- `map.osm`은 OpenStreetMap 내보내기 자료이고 `my_map.xodr`는 변환 자료입니다. 지도 데이터 출처: [© OpenStreetMap contributors](https://www.openstreetmap.org/copyright).

이전 PBB의 파일은 교체됐지만 기존 Git 커밋 기록은 유지합니다.
