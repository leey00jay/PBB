# PBB

## 처음이라면 여기부터

**처음에는 아래 세 폴더 중 본인 담당 폴더만 찾으면 됩니다.** GitHub에서 `src` → `pbb` → 담당 폴더 순서로 클릭하세요.
`src`는 소스 코드, `pbb`는 이 프로젝트의 Python 묶음 이름입니다.

| 내가 만드는 코드 | 올릴 폴더 | 파일 |
| --- | --- | --- |
| 카메라 영상에서 차선·장애물 찾기 → **인지** | [src/pbb/perception](src/pbb/perception/) | 예시: `lane_detector.py` — 아직 없음 |
| 인지 결과를 보고 정지·감속·경로 결정 → **판단** | [src/pbb/planning](src/pbb/planning/) | 예시: `decision.py` — 이 폴더에는 아직 없음 |
| 목표에 맞춰 핸들·가속·브레이크 값 계산 → **제어** | [src/pbb/control](src/pbb/control/) | **[controller.py](src/pbb/control/controller.py) — 공식 PID 연결·설정** |

**기본 주행은 테스트했던 CARLA 공식 오픈소스 `BehaviorAgent`와 `VehiclePIDController`를 사용합니다.**
`control/controller.py`는 공식 제어기를 불러오는 진입점이며, 계산 원본은
[third_party/carla_agents/agents/navigation/controller.py](third_party/carla_agents/agents/navigation/controller.py)에 있습니다.
인지·판단 폴더의 프로젝트용 연결 코드와 새 구조 전체의 주행 연결은 아직 준비 중입니다.
예: 인지가 “빨간불”을 알아냄 → 판단이 “멈춰야 함”을 결정 → 제어가 “제동 값”을 계산합니다.

GitHub는 프로젝트 파일을 저장하고 **누가 무엇을 바꿨는지 기록하는 공동 작업 공간**입니다.
파일이 많아도 모든 파일을 직접 작성하거나 수정할 필요는 없습니다.

<details>
<summary><strong>파일·폴더 전체 설명 펼치기 — 어디에 무엇을 올리는지</strong></summary>

### 1. main의 폴더 지도

```text
PBB/
├── README.md                 ← 지금 읽는 첫 화면 안내
├── src/pbb/
│   ├── perception/           ← 인지 코드를 넣는 곳
│   ├── planning/             ← 판단 코드를 넣는 곳
│   ├── control/controller.py ← 공식 PID 제어기 연결·설정
│   ├── interfaces/           ← 서로 주고받는 데이터 약속
│   ├── runtime/              ← 세 영역을 순서대로 실행하는 연결 코드
│   └── adapters/             ← CARLA 등 외부 프로그램과 연결
├── tests/                    ← 코드가 맞게 동작하는지 확인하는 코드
├── third_party/carla_agents/ ← 테스트에 사용한 공식 오픈소스 원본
├── requirements-control.txt ← 공식 제어기에 필요한 패키지 목록
├── Docs/Project Plan/        ← 프로젝트 계획서
├── docs/architecture.md      ← 코드 분리·연결 방법 설명
├── CONTRIBUTING.md           ← 작업·업로드·병합 절차
└── .github/ 및 설정 파일     ← GitHub 자동 검사와 공동 작업 설정
```

### 2. 코드를 올릴 때 함께 준비할 것

**내가 작성한 코드 + 동작 확인용 테스트 + 짧은 사용 설명**을 올립니다.
예를 들어 판단 코드를 만들었다면 다음과 같습니다.

| 올릴 파일 예시 | 내용 |
| --- | --- |
| `src/pbb/planning/decision.py` | 상황을 보고 주행 상태·목표 속도를 결정하는 코드 |
| `tests/planning/test_decision.py` | 빨간불·장애물·정상 상황에서 원하는 결과가 나오는지 확인하는 코드 |
| `src/pbb/planning/README.md` | 받는 입력, 내보내는 결과, 실행 방법을 설명. 이미 있는 파일에 내용을 추가 |

인지의 테스트는 `tests/perception/`, 제어의 테스트는 `tests/control/`에 넣습니다.
여러 영역을 연결했을 때의 확인 코드는 `tests/integration/`에 넣습니다.
`test_`로 시작하는 `.py` 파일이 자동 테스트 검색 대상입니다.

입력·출력 설명에는 숫자의 단위를 꼭 씁니다. 예를 들어 속도 `15`가 `km/h`인지 `m/s`인지 명시합니다.
인지 결과를 판단에 어떻게 전달할지 등 공통 약속은 `interfaces`에서 먼저 맞춥니다.
기존 실행 코드 한 파일에 인지·판단·제어가 모두 섞여 있다면 폴더만 옮기지 말고, 기능 분리와 실행 확인을 함께 진행합니다.

### 3. 다른 폴더는 언제 사용하는가

| 위치 | 쉬운 설명 | 내가 올릴 내용 |
| --- | --- | --- |
| `src/pbb/interfaces/` | 세 담당자가 쓰는 공통 입력·출력 양식 | 차선 정보·차량 상태·목표 속도 등의 데이터 정의. 팀과 먼저 합의 |
| `src/pbb/runtime/` | 인지 → 판단 → 제어를 차례로 호출하는 진행자 | 전체 실행 코드. 각 담당 기능이 연결될 때 작성 |
| `src/pbb/adapters/` | 외부 프로그램과 우리 코드 사이의 연결부 | CARLA 센서 읽기, 데이터 변환, 차량에 명령 보내기 |
| `tests/` | 코드용 정답 확인 문제 | 정상 입력·오류 입력에 대한 예상 결과 검사 |
| `Docs/Project Plan/README.md` | 프로젝트 계획서 자리 | 목표, 일정, 역할 분담. 현재 문서 내용은 비어 있음 |
| `docs/architecture.md` | 프로젝트 구조 설명서 | 각 영역의 연결 방식이나 공통 규칙이 바뀔 때 수정 |

`Docs`와 `docs`는 현재 별도 폴더입니다. `Docs/Project Plan`은 프로젝트 계획, `docs`는 코드 구조 설명입니다.
새 문서용 폴더를 여러 개 만들기보다 기존 위치를 사용합니다.

### 4. 여러 폴더에 반복되는 파일

| 이름 | 역할 | 처음에 할 일 |
| --- | --- | --- |
| `README.md` | 해당 폴더의 설명서. GitHub에서 폴더를 열면 아래에 보임 | 담당 폴더의 설명만 읽고, 코드를 추가하면 사용법도 적기 |
| `__init__.py` | Python이 이 폴더를 코드 묶음으로 사용할 수 있게 하는 파일 | 현재는 대부분 설명만 있는 준비 파일이므로 그대로 두기 |
| `.py` | Python 프로그램 코드 | 담당 기능과 테스트 작성에 사용 |
| `.md` | 글·표·링크를 적는 Markdown 문서 | 설명이나 계획 작성에 사용 |

### 5. 기본으로 그대로 두는 관리 파일

| 파일 | 역할 |
| --- | --- |
| `.gitignore` | 설치 폴더·임시 파일 등 Git으로 관리하지 않을 경로 목록. 모든 불필요한 파일을 자동 판별해 주는 기능은 아님 |
| `.gitattributes` | 운영체제가 달라도 줄바꿈과 이미지·영상 파일을 일관되게 처리하는 규칙 |
| `.editorconfig` | 들여쓰기·문자 형식 등 편집기 설정 |
| `.github/workflows/ci.yml` | 코드를 올리거나 PR을 만들면 GitHub가 자동으로 실행할 검사 절차 |
| `.github/pull_request_template.md` | 코드를 합쳐 달라고 요청할 때 쓰는 설명 양식 |
| `tests/test_architecture.py` | 인지·판단·제어가 다른 영역의 내부 코드를 직접 가져오지 않는지 확인 |
| `CONTRIBUTING.md` | 팀의 작업 시작·업로드·병합 방법 |
| `requirements-control.txt` | 공식 PID 제어기를 쓰는 데 필요한 CARLA Python API와 NumPy 버전 |
| `third_party/carla_agents/` | 공식 BehaviorAgent·경로 계획·PID 제어 원본과 라이선스. 자체 코드와 구분해 보관 |

이 파일들은 협업을 위해 이미 준비했습니다. 기능 하나를 추가할 때마다 새로 만들 필요는 없습니다.
자동 검사가 통과해도 CARLA 주행 성능까지 검증됐다는 뜻은 아닙니다.

### 6. 무엇을 올리지 않는가

- `.venv/`, `.carla-python/`, `.autonomy-deps/`: 내 컴퓨터에 설치한 실행 환경. 필요한 패키지 이름·버전을 적은 파일로 공유합니다.
- `__pycache__/`, `.pyc`: Python이 실행 중 만든 임시 파일.
- CARLA 설치 폴더 전체, 대용량 원본 영상·데이터셋: 코드 저장소에 통째로 올리지 않고 보관 위치를 문서에 적습니다.
- 비밀번호, 인증 토큰, 비밀값이 들어 있는 `.env`, 개인 키 파일.

설명에 필요한 작은 이미지, 선택한 실험 보고서, 설정 예제는 함께 올릴 수 있습니다.
ZIP으로 프로젝트 전체를 묶기보다 변경한 원본 `.py`와 설명 파일을 올리면 수정 내용을 비교하기 쉽습니다.

### 7. 브랜치와 업로드 — 용어부터

| 용어 | 이 프로젝트에서의 의미 |
| --- | --- |
| 저장소 / Repository | PBB 프로젝트 파일과 변경 기록 전체 |
| `main` | 팀이 합친 공통 기준 버전 |
| 브랜치 / Branch | 공통 버전에서 갈라져 작업하는 별도의 변경 이력. 폴더와는 다름 |
| 커밋 / Commit | 변경사항을 설명과 함께 기록하는 저장 지점 |
| Push | 내 컴퓨터의 커밋을 GitHub로 올리기 |
| Pull Request / PR | 작업한 변경을 `main`에 합쳐 달라는 요청 |
| Merge / 병합 | 검토한 변경을 공통 버전에 합치기 |

**브랜치는 작업 버전이고, 폴더는 그 버전 안에서 파일을 놓는 위치입니다.**
예를 들어 `codex/planning/traffic-light` 브랜치에서도 파일 위치는 `src/pbb/planning/decision.py`입니다.
브랜치 이름과 같은 폴더를 새로 만들지 않습니다.

브라우저에서 업로드하는 기본 순서는 다음과 같습니다. 저장소에 쓰기 권한이 있는 계정으로 로그인합니다.

1. PBB 첫 화면의 브랜치 선택 메뉴에서 `main`을 선택합니다.
2. 담당 폴더로 들어갑니다. 판단이라면 `src` → `pbb` → `planning`입니다.
3. **Add file → Upload files**를 눌러 그 폴더에 넣을 `.py` 파일을 선택합니다.
4. 변경 설명을 적고 **새 브랜치를 만드는 옵션**을 선택합니다. 예: `codex/planning/traffic-light`.
5. 업로드를 확정한 후, 같은 작업 브랜치에서 테스트 파일과 README 설명도 올립니다. 테스트는 `tests/planning` 위치에 올립니다.
6. **Pull requests → New pull request**에서 `base: main`, `compare: 내 작업 브랜치`를 선택합니다.
7. 변경 목적과 확인한 결과를 적고 **Create pull request**를 누릅니다. 검사와 리뷰 후 병합합니다.

이미 같은 기능의 작업 브랜치가 있다면 그 브랜치에서 이어서 작업합니다.
파일 하나마다 브랜치를 만들지 않습니다. 병합이 끝난 뒤 다음 기능은 최신 `main`에서 새로 시작합니다.
자세한 화면 안내: [GitHub 파일 업로드](https://docs.github.com/en/repositories/working-with-files/managing-files/adding-a-file-to-a-repository), [PR 만들기](https://docs.github.com/en/pull-requests/how-tos/create-pull-requests/creating-a-pull-request).

### 8. 다른 브랜치에서 파일이 더 보이는 이유

작성 시점(2026-11-09)에 `main`에는 협업 구조·공식 오픈소스 원본·제어 진입점이 있고, 아래 실행 코드는 별도 작업 브랜치에 있습니다.
GitHub 왼쪽 위의 브랜치 선택이 달라지면 보이는 파일도 달라집니다. 아래 링크는 해당 브랜치로 바로 이동합니다.

- [`ye`의 판단 코드](https://github.com/leey00jay/PBB/blob/ye/decision/decision.py): 기존 `decision.py`. 통합할 때 목적지는 `src/pbb/planning/`입니다.
- [`ye`의 판단 설계 문서](https://github.com/leey00jay/PBB/blob/ye/Docs/Project%20Plan/decision/decision_design.md): 상태·입력·출력 설계입니다.
- [CARLA 실행 예제](https://github.com/leey00jay/PBB/tree/codex/integration/carla-autonomy-demo/examples/carla_boundary): 현재 실행 형태를 보존한 예제. `src/pbb`로 기능을 나누는 작업은 별도로 진행합니다.

CARLA 예제 안의 파일은 다음 역할입니다. 실행 준비는 그 폴더의 `README.md`를 먼저 읽습니다.

| 예제 안의 파일·폴더 | 역할 |
| --- | --- |
| `README.md`, `USAGE.ko.md` | 설치·실행·결과 확인 설명 |
| `run_open_source_autonomy.sh` | 자동주행을 시작하는 실행 스크립트 |
| `open_source_autonomy.py` | 경계 인지와 CARLA 에이전트를 연결해 자동주행 실행 |
| `road_boundary_recovery/road_boundary_recovery.py` | 영상에서 도로 경계 검출 |
| `road_boundary_recovery/README.txt` | 검출 코드의 원래 설명 |
| `boundary_controller.py` | 검출 위치를 목표점으로 바꾸고 조향·속도 제어값 계산 |
| `autonomous_boundary_drive.py` | 정지·출발·직진 주행 반복 시험 |
| `run_boundary_carla.py` | CARLA 카메라의 경계 검출 결과를 확인하는 화면 |
| `test_boundary_controller.py` | 제어 함수의 결과 검사 |
| `analyze_autonomous_test.py`, `analyze_open_source_run.py` | 주행 기록을 분석해 보고서·그래프 생성 |
| `launch_carla.py`, `stop_carla.py` | CARLA 서버 시작·종료 |
| `configure_carla.py` | CARLA 시작 지도·프레임 등 설정 |
| `setup_carla_swap.sh` | Linux 스왑 공간 설정용 선택 도구. 일반 주행 때 실행할 파일이 아님 |
| `requirements-autonomy.txt` | 추가 설치해야 할 Python 패키지 이름·버전 |
| `.gitignore` | 이 예제에서 생기는 영상·실행 기록 등의 자동 업로드 제외 목록 |
| `third_party/carla_agents/` | 외부에서 가져온 CARLA 공식 코드. 담당자 코드와 구분해 보관 |
| `third_party/carla_agents/LICENSE`, `UPSTREAM.json` | 외부 코드의 사용 조건·출처·원본 정보 |
| `validation/날짜-시간/` | 선택해 보관한 실험 결과. 실행 프로그램이 아님 |

`third_party` 안의 `basic_agent.py`, `behavior_agent.py`, `constant_velocity_agent.py`는 주행 에이전트,
`behavior_types.py`는 행동 설정, `global_route_planner.py`와 `local_planner.py`는 경로 계획,
`controller.py`는 제어 함수입니다. `tools/misc.py`와 `hints.py`는 보조 기능이고
`requirements.txt`는 패키지 목록입니다. 외부 원본을 고칠 때는 프로젝트 자체 코드와 변경을 구분합니다.

`validation` 안의 `검증결과.txt`는 사람이 읽는 보고서, `summary.json`은 요약 수치,
`metadata.json`은 실행 조건, `verification.json`은 기록 검증 결과, `frames.jsonl.gz`는 압축한 상세 기록,
`metrics.png`는 그래프, `.jpg` 파일은 대표 주행 장면입니다. 처음에는 보고서와 그래프만 보면 됩니다.

</details>

---

인지·판단·제어를 독립적으로 개발하고 `main`에서 통합하는 프로젝트입니다.
협업 구조와 자동 검사를 갖추었고, `src/pbb/control/controller.py`에서 공식 CARLA PID 제어기를 사용합니다.
기존 주행 검증은 공식 BehaviorAgent에 영상 보조 조향을 연결한 구성으로 수행했습니다.
자체 인지 코드와 실행 연결을 역할별 폴더에 정리하는 작업은 후속 단계입니다.

## 코드 위치

| 역할 | 경로 | 책임 |
| --- | --- | --- |
| 인지 | `src/pbb/perception/` | 카메라·센서 처리, 차선·장애물·신호 인식 |
| 판단 | `src/pbb/planning/` | 주행 상태, 경로, 목표 속도 결정 |
| 제어 | `src/pbb/control/` | 목표를 조향·가속·제동 명령으로 변환 |
| 공통 규격 | `src/pbb/interfaces/` | 세 모듈이 주고받는 데이터 형식 |
| 실행 연결 | `src/pbb/runtime/` | 인지 → 판단 → 제어 호출 및 연결 |
| 외부 연동 | `src/pbb/adapters/` | CARLA 등 외부 API 입출력 |
| 테스트 | `tests/` | 역할별 테스트와 통합 검사 |

각 모듈은 다른 모듈의 내부 코드를 직접 가져오지 않고 공통 규격을 사용합니다.
실제 연결은 `runtime`에서 담당합니다. 자세한 규칙은 [구조 설명](docs/architecture.md)을 참고하세요.

## 개발과 병합

`main`은 통합 기준입니다. 최신 `main`에서 **작업 하나당 브랜치 하나**를 만들고, 테스트를 통과한 PR을 `main`으로 합칩니다.

```text
main → codex/perception/lane-detection → PR → main
main → codex/planning/traffic-light    → PR → main
main → codex/control/speed-controller  → PR → main
```

`perception`, `planning`, `control`이라는 영구 브랜치를 따로 누적하지 않습니다.
공통 규격 변경을 먼저 합친 뒤 각 기능을 작은 PR로 병합하면 충돌과 통합 지연을 줄일 수 있습니다.
[작업 시작·동기화·병합 절차](CONTRIBUTING.md)를 따르세요.

## 검사 실행

Python 3.10 또는 3.12의 가상환경에서 저장소 최상위 폴더 기준으로 실행합니다.
CARLA Python API를 설치하지만 검사 중 시뮬레이터 서버를 실행하지는 않습니다.

```bash
python3 -m pip install -r requirements-control.txt
PYTHONPATH=src:third_party/carla_agents python3 -m unittest discover -s tests -v
python3 -m compileall -q src tests third_party/carla_agents
```

GitHub Actions도 PR마다 같은 검사를 실행합니다. 모듈 간 import 경계, 공식 PID의 동작, 원본 코드 해시를 확인합니다.
주행 성능 검증을 대신하지 않습니다. 알고리즘을 추가할 때 해당 역할의 동작 테스트와 통합 테스트를 함께 추가합니다.

## 기존 자료

- [프로젝트 계획](Docs/Project%20Plan/README.md)
- 기존 로컬 촬영 자료와 실행 스크립트는 이번 구조 변경에 포함하지 않았습니다.
- `main` 자동 업로드를 사용하던 환경은 [전환 절차](CONTRIBUTING.md#기존-자동-업로드에서-전환)를 먼저 확인하세요.
