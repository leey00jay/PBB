# 제어 파트 — control

**판단 파트가 정한 경로와 목표 속도를 실제 차량 움직임으로 만듭니다.**
카메라에서 차선을 찾거나 차로를 선택하는 일은 인지·판단 파트가 담당합니다.

## 담당 파일

| 파일 | 역할 |
|---|---|
| `vehicle_controller.py` | 현재 카메라 모드의 액셀·브레이크·조향 제한과 제어 명령 계산 |
| `pure_pursuit.py` | 경로를 따라갈 조향각 계산, 원본 교재의 PID 제어기 |
| `get_target_point.py` | 경로에서 Pure Pursuit 목표점 선택 |

**이제 액셀 상한 0.4와 속도 계산을 수정할 곳은 `vehicle_controller.py`입니다.**
`pure_pursuit.py` 위쪽의 `param_Kp`, `param_Ki`, `param_Kd`는 현재 `--ld` 카메라 모드의 속도 계산에 사용되지 않습니다.

## 현재 동작

- 입력 속도는 m/s, 목표 속도는 km/h입니다. 계산할 때 목표를 3.6으로 나눕니다.
- 기존 비례 속도 제어를 유지하며 액셀 상한은 0.4입니다. PI/PID 방식으로 새로 개선한 상태가 아닙니다.
- 조향 명령 범위와 시간에 따른 조향 변화량 제한을 유지합니다.
- 판단이 정지를 지시하거나 Pure Pursuit 목표점이 없으면 제동합니다.
- 실제 20km/h 유지 문제를 해결한 것은 아닙니다. 이번 변경은 담당 코드의 분리입니다.

## 다음 작업

1. 목표 속도, 실제 속도, 액셀, 브레이크, 판단 상태를 같은 시각 기준으로 기록합니다.
2. 목표 미달의 원인이 속도 제어인지, 판단의 반복 정지인지 구분합니다.
3. 필요한 경우 PI/PID 속도 제어를 도입하고 적분값 누적과 정지 후 재출발 처리를 검증합니다.
4. 조향 흔들림, 곡선 이탈, 목표 경로가 사라졌을 때의 제동을 검사합니다.

## 전달 형식

`VehicleController.update(plan, speed, dt)` → [ControlCommand](../interfaces.py)

- `speed`: 실제 속도 m/s. `dt`: 시뮬레이션 시간 간격, 초.
- `throttle`, `brake`: 0~1. `steer`: -1~1 범위의 정규화된 명령.
- `target_kmh`, `status`: 제어까지 반영된 최종 목표와 상태. 조향 목표점을 못 찾으면 `NO STEERING TARGET - STOP`으로 보고합니다.
- CARLA에 실제 명령을 보내는 것은 실행 파일의 역할입니다. 제어 계산 자체는 CARLA 없이 시험할 수 있습니다.

## 검사

PBB의 `.venv`를 활성화한 뒤 `aad` 폴더에서:

```bash
python -m unittest code.tests.control.test_vehicle_controller -v
python -m unittest code.tests.control.test_lane_following -v
```
