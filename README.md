# PBB

인지·판단·제어를 독립적으로 개발하고 `main`에서 통합하는 프로젝트입니다.
현재 저장소는 협업 구조와 자동 검사부터 준비한 상태입니다. 실제 알고리즘과 CARLA 실행 코드는 별도 작업 PR로 추가합니다.

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

GitHub Actions도 PR마다 같은 검사를 실행합니다. 현재 검사는 모듈 간 import 경계를 확인하며,
주행 성능 검증을 대신하지 않습니다. 알고리즘을 추가할 때 해당 역할의 동작 테스트와 통합 테스트를 함께 추가합니다.

## 기존 자료

- [프로젝트 계획](Docs/Project%20Plan/README.md)
- 기존 로컬 촬영 자료와 실행 스크립트는 이번 구조 변경에 포함하지 않았습니다.
- `main` 자동 업로드를 사용하던 환경은 [전환 절차](CONTRIBUTING.md#기존-자동-업로드에서-전환)를 먼저 확인하세요.
