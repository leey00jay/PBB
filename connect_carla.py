import carla

# 내 컴퓨터의 2000번 포트에서 실행 중인 CARLA에 연결
client = carla.Client("localhost", 2000)

# 서버 응답을 최대 10초 기다림
client.set_timeout(10.0)

# 현재 시뮬레이션 세계 가져오기
world = client.get_world()

print("서버 버전:", client.get_server_version())
print("API 버전:", client.get_client_version())
print("현재 지도:", world.get_map().name)

# 날씨를 비 오는 낮으로 변경
world.set_weather(carla.WeatherParameters.ClearNoon)
print("날씨 변경 완료")