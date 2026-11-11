# 인지·판단·제어 협업 방법

현재 저장소는 Town10 실습 중인 코드를 파트별로 나눈 상태입니다.
폴더 분리 자체가 새 검출기 연결, 20km/h 유지, 교차로 주행의 완성을 뜻하지는 않습니다.

## 담당 범위

| 파트 | 주로 수정할 폴더 | 검사 |
|---|---|---|
| 인지 | `aad/code/solutions/perception` | `aad/code/tests/perception` |
| 판단 | `aad/code/solutions/decision` | `aad/code/tests/decision` |
| 제어 | `aad/code/solutions/control` | `aad/code/tests/control` |
| 연결 담당 | `aad/code/solutions/runtime`, CARLA 실행 파일 | 전체 검사, 실제 실행 확인 |

`interfaces.py`의 데이터 형식을 바꾸면 양쪽 담당자가 함께 반영해야 합니다.
파일을 각자 복사해서 별도의 전체 주행 프로그램을 만들기보다, 담당 모듈을 수정하고 같은 실행 파일로 확인하세요.
`lane_detection`의 호환 파일 대신 위 담당 폴더의 실제 코드를 수정하세요.

## 작업 시작

이전 PBB 구조가 남은 오래된 브랜치를 그대로 이어 쓰지 말고, 최신 `main`에서 작업 브랜치를 만드세요.
아래 예시는 인지 작업용입니다. 다른 파트는 브랜치 이름을 `codex/decision-work` 또는 `codex/control-work`처럼 구분하면 됩니다.
이미 수정 중인 파일이 있다면 먼저 그 작업을 보관한 뒤 브랜치를 전환하세요.

```bash
cd ~/Desktop/PBB
git switch main
git pull --ff-only
git switch -c codex/perception-work
```

## 검사와 업로드

```bash
source .venv/bin/activate
cd aad
python -m unittest discover -s code/tests -t . -p 'test_*.py' -v
cd ..
git diff
git status
```

수정한 파일을 확인해서 커밋하고 작업 브랜치를 올린 뒤, GitHub에서 `main`을 대상으로 Pull Request를 만드세요.
PR에는 바뀐 동작, 확인한 영상·주행 조건, 검사 결과를 적습니다.
`main`과 PR에는 GitHub 자동 검사가 실행됩니다.

현재 검사는 합성 입력 기반의 동작 검사입니다. 단위 테스트 통과와 실제 도로 주행 성능은 별도로 확인해야 합니다.
카메라 기반 성능을 평가할 때 지도 정답을 사용했다면, 주행 판단에 사용했는지 평가에만 사용했는지를 기록하세요.
