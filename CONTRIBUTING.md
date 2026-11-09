# 협업 절차

## 작업 시작

작업 폴더에 수정 중인 파일이 있으면 먼저 현재 작업 브랜치에 명시적으로 커밋합니다.
다른 작업의 파일까지 함께 올리지 않습니다. 아래 예시의 영역과 작업 이름을 바꿔 사용하세요.

```bash
git switch main
git pull --ff-only origin main
git switch -c codex/perception/lane-detection
```

브랜치 예시:

| 작업 | 브랜치 |
| --- | --- |
| 인지 | `codex/perception/lane-detection` |
| 판단 | `codex/planning/traffic-light` |
| 제어 | `codex/control/speed-controller` |
| 공통 데이터 | `codex/interfaces/observation-v1` |
| 통합·설정 | `codex/integration/carla-loop` |

브랜치는 `main` 아래에 중첩되는 폴더가 아니라 같은 저장소의 작업 이력입니다.
모든 브랜치에는 전체 프로젝트가 있고, 담당 폴더 중심으로 수정합니다.
영역별 영구 브랜치나 별도 `develop` 브랜치는 기본으로 두지 않습니다.

## 수정 범위와 의존성

- 인지·판단·제어의 구현과 테스트를 같은 PR에 포함합니다.
- 다른 영역의 내부 함수를 직접 import하지 않습니다. 공통 형식은 `interfaces`, 연결은 `runtime`에 둡니다.
- 공통 형식을 바꿀 때는 단위, 좌표계, 시간 기준, 누락값 의미와 영향을 PR에 씁니다.
- 호환되는 새 필드를 먼저 추가하고, 소비하는 모듈을 옮긴 뒤, 다음 PR에서 구형 필드를 제거합니다.
- 큰 폴더 이동·전체 포맷 변경을 기능 변경과 한 PR에 섞지 않습니다.
- 의존성 파일·공통 설정·실행 파일은 여러 사람이 동시에 고치지 않도록 작업 시작 시 범위를 공유합니다.

## PR 올리기

```bash
PYTHONPATH=src python3 -m unittest discover -s tests -v
python3 -m compileall -q src tests
git diff --check
git status --short
# 실제 수정한 파일만 지정합니다.
git add src/pbb/perception tests/perception
git commit -m "Add lane detection"
git push -u origin HEAD
```

GitHub에서 base를 `main`으로 선택해 PR을 만들고, 변경 목적·공통 규격 영향·검증 결과를 작성합니다.
검증 전에는 Draft 상태를 사용합니다. 한 PR에는 독립적으로 검토·병합할 수 있는 작업 하나를 담습니다.

## 최신 main 반영

작업 브랜치를 공유한 이후에는 이력을 재작성하지 않는 merge를 기본으로 사용합니다.
작업 폴더의 변경을 먼저 커밋한 뒤 실행하세요.

```bash
git fetch origin
git merge origin/main
# 충돌이 있으면 수정 후 해당 파일을 git add하고 git commit합니다.
PYTHONPATH=src python3 -m unittest discover -s tests -v
python3 -m compileall -q src tests
git push
```

충돌 파일을 한쪽 내용으로 통째로 덮어쓰지 말고, 변경 의도와 연결 테스트를 확인합니다.
일상 작업에서 `git push --force`는 사용하지 않습니다.

## 병합과 다음 작업

1. 공통 규격 PR이 있다면 먼저 병합합니다.
2. 인지·판단·제어의 독립 PR은 준비된 순서로 병합합니다. 순서를 고정하지 않습니다.
3. 최신 `main`을 반영하고 CI가 통과했는지 확인합니다. 알고리즘 변경은 관련 동작 테스트도 필요합니다.
4. 동료가 있는 작업은 리뷰 후 GitHub의 **Squash and merge**로 병합합니다.
5. 병합이 끝난 브랜치는 GitHub에서 삭제하고, 다음 작업은 최신 `main`에서 새로 시작합니다.

Squash한 브랜치에 다음 작업을 계속 쌓지 않습니다.
로컬 브랜치는 작업 유실이 없는지 확인한 뒤 정리하며, 강제 삭제를 자동화하지 않습니다.

## GitHub에서 설정할 규칙

아래는 관리자 설정의 목표이며, 파일을 추가하는 것만으로 활성화되지는 않습니다.

- 기본 브랜치: `main`
- `main`에는 PR을 통해 병합하도록 ruleset 또는 branch protection 설정
- 필수 검사: `checks (3.10)`, `checks (3.12)` — 첫 CI 실행 후 실제 표시 이름 확인
- 최신 `main`에 대한 검사 및 미해결 리뷰 대화 해결 요구
- 팀 작업이면 승인 1명 요구. 혼자 작업하는 저장소에는 본인 승인 불가로 생기는 잠금을 피하도록 0명 설정
- 강제 push와 브랜치 삭제 제한
- Squash merge 사용 및 병합 완료 브랜치 자동 삭제

담당자 계정을 확인한 뒤 `CODEOWNERS`를 추가합니다. 계정을 추정해서 넣지 않습니다.
실제 GitHub 설정 적용 여부는 저장소 Settings에서 따로 확인해야 합니다.

## 기존 자동 업로드에서 전환

기존 `pbb-sync.service`는 `~/Desktop/PBB`의 변경 파일을 모두 커밋하여 `main`에 바로 push하도록 만들어졌습니다.
PR 중심 협업과 함께 사용하면 검토 전 코드가 올라가므로 코드 작업 전 중지해야 합니다.

```bash
systemctl --user disable --now pbb-sync.service
```

이번 구조 변경은 기존 바탕화면 폴더의 미커밋 파일을 가져오지 않습니다.
파일을 보존한 상태에서 새 clone을 작업 폴더로 쓰는 방법이 가장 단순합니다.
기존 README와 `.gitignore`가 미커밋 상태라면 기존 폴더에서 바로 pull할 때 충돌할 수 있습니다.
보존할 자료는 별도 작업 브랜치에서 필요한 파일만 선택해서 추가합니다.

## 참고

- [GitHub Flow](https://docs.github.com/en/get-started/using-github/github-flow)
- [PR 병합 방식](https://docs.github.com/en/pull-requests/reference/pull-request-merges)
- [브랜치 보호](https://docs.github.com/en/repositories/configuring-branches-and-merges-in-your-repository/managing-protected-branches/about-protected-branches)
