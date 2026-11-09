# 오픈소스 자율주행 실행

CARLA 공식 **BehaviorAgent 0.9.16 (MIT)** 를 적용했습니다. 자동 경로 계획, 신호 대기, 교차로 회전과 반복 주행을 담당합니다. 첨부한 `BoundaryTracker` 검출값도 적합한 직선 구간에서 실제 조향에 사용합니다.

## 실행

먼저 [환경 준비](README.md#환경-준비)를 완료하세요. 터미널에서 실행하면 CARLA 서버가 없을 때 자동으로 시작하고, 주행 창이 열리면 자동 출발합니다.

```bash
cd examples/carla_boundary
./run_open_source_autonomy.sh
```

- **Space** 또는 **일시정지 / 계속**: 자동주행 정지·재개
- **Esc**, **주행 종료**, 창 닫기: 차량을 세우고 주행 프로그램 종료
- 목적지에 도착하면 다음 목적지를 계획하며 계속 주행합니다. 수동 조향 키는 필요 없습니다.
- CARLA 서버는 주행 창을 닫아도 유지됩니다. 서버도 끝내려면 `python3 stop_carla.py`를 실행합니다.
- 다른 동기식 CARLA 제어 프로그램과 동시에 실행하면 안 됩니다. 중복 실행은 감지하여 중단합니다.

처음 위치에서 다시 시작하며 180초 영상을 기록하려면:

```bash
./run_open_source_autonomy.sh --reset --record-seconds 180
```

180초 뒤 프로그램까지 종료하려면 `--seconds 180`을 추가합니다. 기본은 처음 150초만 영상을 저장하고 주행은 계속합니다. `--reset`은 이 프로그램이 만든 차량만 초기 위치로 옮깁니다.

첨부 검출 조향을 끄고 공식 에이전트만 비교하려면 `--agent-only`를 추가합니다. 기본 최고 속도는 15km/h, 곡선 접근 시 8km/h로 감속합니다. 속도 상한은 `--speed`로 지정할 수 있습니다(1~25km/h). 검증은 기본 속도 설정에서 수행했습니다.

## 화면과 제어

- `BOUNDARY_VISION`: 첨부 영상 검출로 목표점을 계산해 실제 조향 중
- `BOUNDARY_HELD`: 최근 검출을 0.2초 이내로 유지하여 조향 중
- `BEHAVIOR_AGENT_TURN`: 오픈소스 경로 추종으로 회전·곡선 주행 중
- `BEHAVIOR_AGENT_BRAKE`: 오픈소스 제동 판단 적용 중
- `BEHAVIOR_AGENT_NO_DETECTION` / `DISAGREEMENT`: 검출이 없거나 지도 조향과 크게 달라 공식 에이전트의 조향 사용

빨간불, 경로, 주변 차량·보행자 정보는 **CARLA 시뮬레이터 API**에서 읽습니다. 첨부 검출기는 도로 경계를 찾으며, 신호등 영상 인식기는 아닙니다. Traffic Manager 자동운전은 사용하지 않고 BehaviorAgent와 통합 제어기가 차량 명령을 직접 보냅니다. 신호 주기는 시뮬레이터의 자연 주기를 사용합니다.

## 결과 확인

실행마다 `opensource-runs/날짜-시간/`에 결과가 생성됩니다.

- `drive.mp4`: 전방 영상, 검출선, 조향 담당, 속도, 신호 상태
- `summary.json`: 정지·출발·회전 횟수, 차선 오차, 충돌 기록
- `frames.jsonl`: 실제 적용한 조향·가속·제동과 센서 시각
- `metadata.json`: 사용 버전, 설정, 실행 소스 SHA256
- `exit.json`: 종료 이유와 오류 여부(프로그램 종료 후)

녹화가 끝난 뒤 해당 폴더로 검증 보고서와 그래프를 생성할 수 있습니다.

```bash
python3 analyze_open_source_run.py opensource-runs/날짜-시간
```

일반 도로 중심 오차는 이동 중(1km/h 초과) 교차로를 제외하고 측정합니다. 차량 영역의 차로 이탈은 차체 크기와 방향을 고려한 지도 차로 폭 기준입니다. 충돌·차선 침범 센서는 별도로 기록합니다. 다른 차량/보행자 없는 Town02_Opt의 낮·저속 실행 결과이며, 장애물 회피나 악천후 성능 검증을 포함하지 않습니다.

## 적용한 소스

- 공식 코드: https://github.com/carla-simulator/carla/tree/0.9.16/PythonAPI/carla/agents
- 공식 설명: https://carla.readthedocs.io/en/0.9.16/adv_agents/
- 원본 코드와 MIT 라이선스: `third_party/carla_agents/`
- 버전과 원본 파일 해시: `third_party/carla_agents/UPSTREAM.json`
- 통합 코드: `open_source_autonomy.py`
- 첨부 검출 원본: `road_boundary_recovery/road_boundary_recovery.py` (변경하지 않음)
- 좌표 변환과 영상 조향: `boundary_controller.py`

공식 에이전트 파일은 수정하지 않았고, 별도 상속 클래스에서 낮은 속도에 맞게 PID·곡선 감속·영상 조향을 설정했습니다. 추가 의존성 `networkx==3.4.2`, `shapely==2.0.7`은 `.autonomy-deps/`에 격리하여 설치했습니다. CARLA Python API는 `.carla-python/`, NumPy·OpenCV·Pillow·Tk는 시스템 Python을 사용합니다. 다른 컴퓨터에 옮길 경우 CARLA 서버/API 0.9.16과 시스템 의존성을 먼저 준비하고, pip가 있는 Python에서 `python3 -m pip install --target .autonomy-deps --no-deps -r requirements-autonomy.txt`로 추가 패키지를 설치해야 합니다.
