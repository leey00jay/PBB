
# 프붕붕 자율주행 프로젝트
# 판단 모듈 (Decision Module)

def decide(camera_ok, ttc, lane_confidence, slip_risk):

    # 1. 카메라 입력 상실
    if not camera_ok:
        return "EMERGENCY", 0

    # 2. 충돌 위험 판단
    if ttc is not None and ttc < 2.0:
        return "DANGER", 5

    # 3. 차선 인식 저하 또는 미끄러짐 위험
    if lane_confidence < 0.5 or slip_risk > 0.7:
        return "CAUTION", 15

    # 4. 정상 주행
    return "NORMAL", 30


# 가짜 입력 데이터로 테스트
test_cases = [
    (True, 10.0, 0.95, 0.1),  # 정상
    (True, 8.0, 0.30, 0.1),   # 차선 인식 저하
    (True, 1.5, 0.90, 0.1),   # 충돌 위험
    (False, None, 0.0, 0.0),  # 카메라 고장
    (True, 8.0, 0.90, 0.9)    # 미끄러짐 위험
]

for i, data in enumerate(test_cases, start=1):
    state, speed = decide(*data)

    print(f"테스트 {i}")
    print(f"주행 상태: {state}")
    print(f"목표 속도: {speed} km/h")
    print("-" * 25)
