import carla

client = carla.Client("localhost", 2000)
client.set_timeout(10.0)
world = client.get_world()

# 1. 만들 차량의 모델 선택
blueprint = world.get_blueprint_library().find(
    "vehicle.tesla.model3"
)

# 2. 지도에서 차량을 놓을 수 있는 위치 목록 가져오기
spawn_points = world.get_map().get_spawn_points()

# 3. 비어 있는 위치를 찾아 차량 생성
vehicle = None

for point in spawn_points:
    vehicle = world.try_spawn_actor(blueprint, point)

    if vehicle is not None:
        break

if vehicle is None:
    raise RuntimeError("차량을 생성할 빈 위치가 없습니다.")

try:
    # 4. 차량을 정지 상태로 유지
    vehicle.apply_control(
        carla.VehicleControl(brake=1.0, hand_brake=True)
    )
    world.wait_for_tick()

    # 5. 차량의 현재 위치와 방향 가져오기
    pose = vehicle.get_transform()

    # 6. 관찰 시점을 차량 뒤쪽으로 이동
    spectator = world.get_spectator()

    spectator.set_transform(
        carla.Transform(
            pose.location
            - pose.get_forward_vector() * 6
            + carla.Location(z=3),
            carla.Rotation(pitch=-15, yaw=pose.rotation.yaw),
        )
    )

    print("차량 생성 완료! CARLA 화면을 확인하세요.")
    input("차량을 삭제하려면 이 터미널에서 Enter를 누르세요.")

except KeyboardInterrupt:
    print("\n실습을 중단합니다.")

finally:
    vehicle.destroy()
    print("생성한 차량을 삭제했습니다.")