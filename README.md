# hikvision_camera_ros2

ROS2 패키지: Hikvision HCNetSDK로 카메라/IR 이미지를 받아 두 개의 이미지 토픽으로 발행합니다.

## 다른 PC에서 한 폴더만 clone/복사해서 빌드

이 패키지에는 **sdk/** (incEn + lib) 가 포함되어 있습니다. 이 폴더 전체를 `ros2_ws/src/` 에 넣고 빌드하면 됩니다.

```bash
# 이 패키지(hikvision_camera_ros2) 전체를 ros2_ws/src/ 에 복사한 뒤
cd ~/ros2_ws
source /opt/ros/<distro>/setup.bash
colcon build --packages-select hikvision_camera_ros2
source install/setup.bash
```

`-DHIKVISION_SDK_ROOT` 지정 없이 빌드됩니다. 설치 시 SDK 라이브러리는 `install/lib/hikvision_camera_ros2/sdk_libs/` 에 복사되므로 다른 PC에서도 그대로 실행 가능합니다.

## 토픽

| 토픽 | 메시지 타입 | 설명 |
|------|-------------|------|
| `camera/image_raw` | sensor_msgs/Image | 일반(가시광) 카메라 영상 |
| `camera/ir/image_raw` | sensor_msgs/Image | IR 카메라 영상 |

## 요구사항

- ROS2 (Humble/Jazzy 등)
- HCNetSDK Linux 64비트 (이 패키지는 SDK 폴더 **안**에 있어야 함)
- OpenCV, cv_bridge

## 빌드

1. 이 패키지를 ROS2 워크스페이스로 복사하거나 심볼릭 링크합니다.

   **방법 A:** 패키지를 SDK 폴더 **안**에 두고, SDK 폴더 전체를 워크스페이스에 넣기  
   (예: `~/ros2_ws/src/EN-HCNetSDKV6.1.9.48_build20230410_linux64/` 안에 `hikvision_camera_ros2`가 있음)

   **방법 B:** 패키지만 워크스페이스에 링크한 경우, 빌드 시 SDK 경로를 지정:
   ```bash
   cd ~/ros2_ws/src
   ln -s /path/to/EN-HCNetSDKV6.1.9.48_build20230410_linux64/hikvision_camera_ros2 .
   # 빌드할 때:
   HIKVISION_SDK_ROOT=/path/to/EN-HCNetSDKV6.1.9.48_build20230410_linux64 colcon build --packages-select hikvision_camera_ros2
   ```

2. SDK 라이브러리 경로를 설정합니다.

   ```bash
   export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/hongsi/Downloads/EN-HCNetSDKV6.1.9.48_build20230410_linux64/lib:/home/hongsi/Downloads/EN-HCNetSDKV6.1.9.48_build20230410_linux64/lib/HCNetSDKCom
   ```

3. 빌드합니다.

   ```bash
   cd ~/ros2_ws
   colcon build --packages-select hikvision_camera_ros2
   source install/setup.bash
   ```

## 실행

### 파라미터 파일 사용

`config/camera_params.yaml`에서 IP, 비밀번호, 채널 등을 수정한 뒤:

```bash
# LD_LIBRARY_PATH 설정 (위와 동일)
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/path/to/EN-HCNetSDKV.../lib:/path/to/.../lib/HCNetSDKCom

ros2 run hikvision_camera_ros2 hikvision_camera_node --ros-args --params-file /path/to/hikvision_camera_ros2/config/camera_params.yaml
```

### Launch 파일 사용

```bash
ros2 launch hikvision_camera_ros2 hikvision_camera.launch.py \
  device_ip:=192.168.1.64 \
  password:=qwer1234
```

### 단일 채널 카메라

채널이 1개뿐이면 `channel_ir`를 1로 두거나, IR 토픽만 쓰고 싶으면 `channel_visible`/`channel_ir`를 같은 채널로 두면 됩니다.

## 파라미터

| 파라미터 | 타입 | 기본값 | 설명 |
|----------|------|--------|------|
| device_ip | string | 192.168.1.64 | 카메라 IP |
| port | int | 8000 | SDK 포트 |
| username | string | admin | 로그인 사용자명 |
| password | string | (필수) | 로그인 비밀번호 |
| channel_visible | int | 1 | 가시광 채널 번호 |
| channel_ir | int | 2 | IR 채널 번호 |
| frame_rate | double | 2.0 | 발행 주기 (Hz). JPEG 캡처 한계로 1~3 Hz 권장 |

## 참고

- 이미지는 SDK `NET_DVR_CaptureJPEGPicture`로 주기적으로 캡처해 발행합니다. 고프레임이 필요하면 RealPlay 스트림 + 디코딩 방식으로 확장할 수 있습니다.
- 카메라와 PC가 직접 연결된 경우, 해당 인터페이스에 IP(예: 192.168.1.100)를 부여한 뒤 사용하세요.
