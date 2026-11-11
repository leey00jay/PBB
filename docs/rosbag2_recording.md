# ROS 2 주행 녹화·공유·재생

## 녹화

```bash
~/Desktop/PBB/start_ros2.sh desired_speed_kmh:=80.0 record:=true record_mode:=video
```

화면 녹화 모드는 주행 화면(JPEG)·중앙 원본(PNG)·시계 토픽 3개를 PBB/bags/ros2_town10_날짜_시간/에 저장합니다.
전방 카메라 3개는 주행 화면 안에 함께 보입니다. 프레임 시각과 영상 해상도는 유지합니다.
독립된 좌·우 원본과 중간 인지·판단·제어 데이터까지 필요하면 `record_mode:=all`을 사용합니다.
all은 기존처럼 토픽 14개를 저장하며 디스크 사용량이 더 큽니다. record_mode를 생략한 기본값도 all입니다.
원하는 새 폴더에 기록하려면:

```bash
~/Desktop/PBB/start_ros2.sh desired_speed_kmh:=80.0 record:=true record_mode:=video bag_path:=$HOME/Desktop/PBB/bags/town10_02
```

기존 폴더는 덮어쓰지 않습니다. 종료는 Ctrl+C이며 `Recording stopped`를 기다립니다.

## 확인과 공유

```bash
source /opt/ros/humble/setup.bash
source ~/Desktop/PBB/install/setup.bash
ros2 bag info ~/Desktop/PBB/bags/town10_02
cd ~/Desktop/PBB/bags
zip -r town10_02.zip town10_02
```

`.db3`와 `metadata.yaml`이 포함된 폴더 전체를 압축해 GitHub Releases에 올립니다.
bags는 Git에서 제외되어 일반 git push의 대상이 아닙니다. 촬영마다 새 폴더·압축 이름을 사용하세요.
실행에 사용한 코드 커밋을 릴리즈 설명에 적으면 영상과 코드를 연결할 수 있습니다.

## 재생

실제 주행을 종료하고, 실제 폴더 이름으로 다음을 실행합니다.

```bash
source /opt/ros/humble/setup.bash
source ~/Desktop/PBB/install/setup.bash
ros2 bag play ~/Desktop/PBB/bags/town10_02 --loop
```

다른 터미널에서:

```bash
source /opt/ros/humble/setup.bash
ros2 run rqt_image_view rqt_image_view
```

`/pbb/screen/image/compressed`는 주행 화면, `/pbb/front/image/compressed`는 중앙 원본입니다.
플러그인이 없다면 `sudo apt install ros-humble-compressed-image-transport ros-humble-rqt-image-view`로 설치합니다.
좌·중앙·우·추적 원본도 `/pbb/camera/left/image_raw` 등 별도 토픽으로 있습니다.

팀원이 사용자 정의 메시지를 읽으려면 PBB 코드를 받아 `./build_ros2.sh`, `source install/setup.bash`를 실행합니다.
표준 영상만 재생하려면 bag play에 `--topics /pbb/screen/image/compressed /pbb/front/image/compressed`를 붙입니다.
과거 영상 2개짜리 bag도 같은 두 토픽으로 재생할 수 있습니다.
bag play는 메시지를 발행합니다. 영상 창이나 CARLA 차량을 자동으로 띄우지는 않습니다.
전체 토픽·단위는 [ROS 안내](ros2_launch.md)를 참고하세요.
