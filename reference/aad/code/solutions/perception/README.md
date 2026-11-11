# 인지 파트 — perception

**카메라 영상에서 무엇이 보이는지 판단할 근거를 만듭니다.**
액셀·브레이크·조향 명령은 이 폴더에서 만들지 않습니다.

## 담당 파일

| 파일 | 역할 |
|---|---|
| `lane_detector.py` | 기존 신경망으로 배경·왼쪽·오른쪽 차선 확률 계산 |
| `camera_geometry.py` | 카메라 투영과 지면 좌표 계산 |
| `lane_perception.py` | 차선 확률에서 유효한 경계 곡선 추출, 인식 화면 생성 |
| `road_boundary_recovery.py` | 전달받은 노란 중앙선·오른쪽 도로 가장자리 검출 및 추적 |

모델 파일은 교재와 기존 실행 경로 호환을 위해 `../lane_detection/fastai_model.pth`에 둡니다.
`../lane_detection/`의 같은 이름 Python 파일들은 이전 import를 연결하는 호환 파일입니다.
실제 인지 로직은 **이 폴더**에서 수정하세요.

## 현재 연결 상태

```text
전방 RGB 영상 → lane_detector → lane_perception → LaneObservation → 판단
전방 BGR 영상 → road_boundary_recovery → 오른쪽 분석 패널
```

새 도로 경계 코드의 결과는 아직 판단·제어로 전달하지 않습니다.
노란 중앙선과 오른쪽 도로 끝이 보인다고 현재 차로의 양쪽 경계를 찾은 것은 아닙니다.
특히 Town10의 여러 차로 구간에서는 둘 사이의 중간을 내 차로 중심으로 사용할 수 없습니다.

## 다음 작업

1. 고정한 Town10 구간에서 좌우 차선, 중앙선, 도로 가장자리를 구분해 검출 결과를 확인합니다.
2. 새 검출기에 필요한 흰 차선 검출 또는 현재 차로 선택 방법을 보완합니다.
3. 새 경계의 픽셀 위치를 카메라 설정에 맞춰 미터 단위로 변환합니다.
4. `tracked`, `held`, `lost`와 결과가 얼마나 오래됐는지 구분해서 판단 파트에 전달할 방법을 정합니다.
5. 인식 실패 시 없는 경계를 0으로 만들어 보내지 않고, 실패 상태를 명확히 전달합니다.

## 전달 형식

현재 `LanePerception.observe(rgb)`는 [LaneObservation](../interfaces.py)을 반환합니다.

- `left`, `right`: 경계 곡선 `y(x)` 또는 `None`. 카메라 지면 기준 미터, x는 전방, y는 왼쪽입니다.
- `overlay`: 인식 표시를 그린 RGB 영상. 원본 입력 영상은 변경하지 않습니다.
- 새 검출기의 `center_error_px`는 픽셀 값입니다. 위 곡선이나 제어 명령으로 그대로 넘길 수 없습니다.

## 검사

PBB의 `.venv`를 활성화한 뒤 `aad` 폴더에서 실행합니다.

```bash
python -m unittest code.tests.perception.test_lane_perception -v
```

실제 영상에서도 곡선이 올바른 경계를 따라가는지 확인해야 합니다. 이 단위 테스트만으로 Town10 전체 검출 성능이 검증되지는 않습니다.
