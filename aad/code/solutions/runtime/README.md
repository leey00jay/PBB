# 파트 연결 — runtime

`lane_following.py`의 `LaneFollower`가 다음 순서로 호출합니다.

```text
LanePerception.observe(영상)
        ↓ LaneObservation
LanePlanner.plan(인지 결과, 요청 속도)
        ↓ DrivePlan
VehicleController.update(계획, 현재 속도, 시간 간격)
        ↓ ControlCommand
기존 CARLA 실행 파일에서 vehicle.apply_control 호출
```

이 파일에는 각 파트의 알고리즘을 다시 넣지 않습니다. 연결과 기존 화면용 결과 조합을 담당합니다.
공통 데이터 형식은 [interfaces.py](../interfaces.py)에 있습니다.

실행 진입점은 기존과 같습니다.

- [carla_sim_boundary.py](../../tests/control/carla_sim_boundary.py): 기존 차선 유지 + 새 도로 경계 분석 패널.
- [carla_sim.py](../../tests/control/carla_sim.py): 기존 차선 유지 화면.

새 도로 경계 검출은 아직 오른쪽 패널에만 연결돼 있습니다. 이번 폴더 분리는 새 검출기를 조향에 연결하는 기능 변경을 포함하지 않습니다.
맵, 차량, 카메라 설정과 목표 속도 설정은 실행 파일에 있습니다.
