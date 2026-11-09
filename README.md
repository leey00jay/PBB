# PBB

인지·판단·제어를 독립적으로 개발하고 `main`에서 통합하는 프로젝트입니다.
협업용 모듈 구조와 자동 검사를 갖추고 있으며, 검증한 CARLA 자동주행 코드는 독립 실행 예제로 제공합니다.

## CARLA 자동주행 예제

[경계 검출 + 공식 BehaviorAgent 실행 방법](examples/carla_boundary/README.md)에서 코드와 검증 결과를 확인할 수 있습니다.
수동 조작 없이 신호 대기·재출발과 교차로 회전을 수행하며, 첨부 검출값을 실제 조향에 연결합니다.
180초 검증에서 신호 정지·출발 4회, 교차로 회전 3회, 충돌·차선 침범 0회를 기록했습니다.
신호와 경로는 CARLA API를 사용하며, 전체 조건은 예제의 검증 보고서에 명시합니다.

이 예제는 검증 당시 실행 구조를 보존합니다. `src/pbb`의 공통 규격과 모듈 분리는 [이관 순서](docs/architecture.md#기존-코드-이관-순서)에 따라 별도로 진행합니다.

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

Python 3.10 이상에서 저장소 최상위 폴더에서 실행합니다. 기본 검사에는 추가 패키지가 필요하지 않습니다.

```bash
PYTHONPATH=src python3 -m unittest discover -s tests -v
python3 -m compileall -q src tests
```

GitHub Actions는 PR마다 위 검사와 CARLA 예제의 제어 함수 테스트·문법 검사를 실행합니다.
CI에는 CARLA 시뮬레이터 주행이 포함되지 않습니다. 알고리즘을 추가할 때 해당 역할의 동작 테스트와 통합 테스트를 함께 추가합니다.

## 기존 자료

- [프로젝트 계획](Docs/Project%20Plan/README.md)
- 전체 녹화 영상과 설치 패키지는 Git에 포함하지 않습니다. 자동주행 코드·실행 방법·선별 검증 자료는 위 예제에 포함합니다.
- `main` 자동 업로드를 사용하던 환경은 [전환 절차](CONTRIBUTING.md#기존-자동-업로드에서-전환)를 먼저 확인하세요.
