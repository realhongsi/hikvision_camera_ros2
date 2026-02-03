## 폴더 구조

```
hikvision_camera_ros2_deploy/
├── README.md           # 이 파일
├── sdk/                # Hikvision SDK (incEn, lib)
│   ├── incEn/          # 헤더 (HCNetSDK.h 등)
│   └── lib/            # 공유 라이브러리 (.so) 및 HCNetSDKCom
└── src/
    └── hikvision_camera_ros2/   # ROS2 패키지
        ├── CMakeLists.txt
        ├── package.xml
        ├── config/
        ├── launch/
        └── src/
```

- **sdk/** : 빌드·실행에 필요한 SDK만 포함 (incEn, lib). 기존 SDK 폴더는 그대로 두고 여기만 복사한 것.
- **src/hikvision_camera_ros2/** : ros2_ws에 있던 패키지 전체 복사 (RTSP, 온도 토픽, ISAPI 폴백, 백그라운드 온도 등 반영).

## 요구 사항

- ROS2 (Humble/Jazzy 등)
- `libcurl4-openssl-dev` (온도 토픽·ISAPI 사용 시)
- OpenCV, cv_bridge, sensor_msgs, std_msgs

```bash
sudo apt install libcurl4-openssl-dev
```

## 빌드

이 폴더(`hikvision_camera_ros2_deploy`)를 **어디든** 복사한 뒤, 그 안에서:

```bash
cd /path/to/hikvision_camera_ros2_deploy
source /opt/ros/<distro>/setup.bash   # 예: humble, jazzy
colcon build --packages-select hikvision_camera_ros2
```

SDK 경로는 **자동**으로 `sdk/` 를 사용합니다 (패키지가 `src/hikvision_camera_ros2` 에 있고 `sdk/` 가 같은 상위에 있으면 됨).  
다른 위치에 SDK를 두었다면:

```bash
colcon build --packages-select hikvision_camera_ros2 --cmake-args -DHIKVISION_SDK_ROOT=/path/to/sdk
```

## 실행

```bash
source install/setup.bash
ros2 launch hikvision_camera_ros2 hikvision_camera.launch.py
```

또는 파라미터 파일 경로를 지정:

```bash
ros2 run hikvision_camera_ros2 hikvision_camera_node --ros-args --params-file src/hikvision_camera_ros2/config/camera_params.yaml
```

**config 수정:** `src/hikvision_camera_ros2/config/camera_params.yaml` 에서 `device_ip`, `password`, `channel_ir` 등을 카메라에 맞게 설정하세요.

## 토픽

- `/camera/image_raw` : 가시광 이미지
- `/camera/ir/image_raw` : 열화상(IR) 이미지
- `/camera/ir/temperature_info` : 온도 정보 (min, max, avg 등, layout.label 포함). `publish_temperature_topic: true` 일 때만 발행.


