# 제어 — CARLA 공식 PID 제어기 사용

**기본 제어는 테스트했던 CARLA 0.9.16의 공식 `VehiclePIDController`입니다.**
[controller.py](controller.py)는 그 클래스를 가져오고 기존 주행 시험의 PID 설정을 적용하는 진입점입니다.
계산 원본은 [공식 controller.py](../../../third_party/carla_agents/agents/navigation/controller.py)에 있으며 수정하지 않았습니다.

| 코드 | 역할 |
| --- | --- |
| `VehiclePIDController` | 횡·종방향 PID를 조합해 조향·가속·제동 명령 계산 |
| `PIDLateralController` | 목표 경로점을 향하도록 조향 값 계산 |
| `PIDLongitudinalController` | 목표 속도를 따르도록 가속·제동 계산 |
| `create_controller(vehicle)` | 기존 시험 설정으로 공식 제어기를 생성 |

공식 실행 흐름은 `BehaviorAgent → LocalPlanner → VehiclePIDController`입니다.
`BehaviorAgent`를 사용하는 실행에서는 에이전트가 이미 제어기를 갖고 있으므로 제어기를 추가로 만들어 중복 적용하지 않습니다.
신호 판단·정지 동작은 에이전트가 담당합니다. `create_controller`는 제어기를 별도로 사용하거나 시험할 때의 진입점입니다.

기존 검증은 **공식 BehaviorAgent에 영상 경계 기반 보조 조향을 연결한 구성**으로 수행했습니다.
`boundary_controller.py`의 `pursuit_steer`는 보조 조향 함수이고, `longitudinal`은 이전 직진 시험용입니다.
이를 공식 PID 제어기의 대체 구현으로 옮기지 않습니다.

## 입력·출력

`controller.run_step(target_speed_kmh, waypoint)`는 **목표 속도 km/h**와 CARLA 경로점을 받아
`carla.VehicleControl`을 반환합니다. 반환값의 `steer`, `throttle`, `brake`가 조향·가속·제동 명령입니다.
차량에 전달하는 `vehicle.apply_control(...)`은 실행 연결에서 수행합니다.

기존 시험 설정: 20 Hz, 가속 상한 0.55, 일반 PID 제동 상한 0.6, 조향 상한 0.8,
횡방향 PID `(K_P=1.2, K_I=0, K_D=0.05)`, 종방향 PID `(K_P=0.35, K_I=0.03, K_D=0)`.
일반 PID 제동과 에이전트의 비상 정지 판단을 구분합니다.

## 검사

Linux에서 저장소 최상위 폴더 기준입니다. 가상환경에 의존성을 설치합니다.

```bash
python3 -m venv .venv
. .venv/bin/activate
python3 -m pip install -r requirements-control.txt
PYTHONPATH=src:third_party/carla_agents python3 -m unittest discover -s tests -v
```

실제 공식 PID 코드와 CARLA Python API의 자료형을 사용하며 차량 상태만 테스트용으로 제공합니다.
CARLA 서버를 띄우지 않으므로 새 구조로 전체 주행을 재검증한 것은 아닙니다.
기존 주행 실행 파일과 결과는 [실행 브랜치](https://github.com/leey00jay/PBB/tree/codex/integration/carla-autonomy-demo/examples/carla_boundary)에 보존되어 있습니다.

원본의 MIT [라이선스](../../../third_party/carla_agents/LICENSE)와
[출처·원본 해시](../../../third_party/carla_agents/UPSTREAM.json)를 함께 보관합니다.
[공식 CARLA 설명](https://carla.readthedocs.io/en/0.9.16/adv_agents/)에서 내부 연결 구조를 확인할 수 있습니다.
