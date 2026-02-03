cmake_minimum_required(VERSION 3.8)
project(hikvision_camera_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# SDK path: auto-detect when in deploy layout (src/hikvision_camera_ros2 + sdk/), else -D or env
set(HIKVISION_SDK_ROOT "" CACHE PATH "Hikvision SDK root")
if(NOT HIKVISION_SDK_ROOT AND DEFINED ENV{HIKVISION_SDK_ROOT})
  set(HIKVISION_SDK_ROOT "$ENV{HIKVISION_SDK_ROOT}" CACHE PATH "" FORCE)
endif()
if(NOT HIKVISION_SDK_ROOT)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../../sdk/incEn/HCNetSDK.h")
    set(HIKVISION_SDK_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../sdk" CACHE PATH "" FORCE)
  else()
    set(HIKVISION_SDK_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/.." CACHE PATH "" FORCE)
  endif()
endif()
set(HIKVISION_INCLUDE_DIR "${HIKVISION_SDK_ROOT}/incEn")
set(HIKVISION_LIB_DIR "${HIKVISION_SDK_ROOT}/lib")

if(NOT EXISTS "${HIKVISION_INCLUDE_DIR}/HCNetSDK.h")
  message(FATAL_ERROR
    "Hikvision SDK not found at: ${HIKVISION_SDK_ROOT}\n"
    "  In deploy layout, ensure sdk/ (incEn, lib) is next to src/. Or:\n"
    "  colcon build --packages-select hikvision_camera_ros2 --cmake-args -DHIKVISION_SDK_ROOT=/path/to/sdk")
endif()
if(NOT EXISTS "${HIKVISION_LIB_DIR}/libhcnetsdk.so")
  message(FATAL_ERROR "Hikvision SDK lib not found at: ${HIKVISION_LIB_DIR}")
endif()
message(STATUS "Hikvision SDK: ${HIKVISION_SDK_ROOT}")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(CURL REQUIRED)

include_directories(
  include
  ${HIKVISION_INCLUDE_DIR}
)

add_executable(hikvision_camera_node src/hikvision_camera_node.cpp)
ament_target_dependencies(hikvision_camera_node rclcpp sensor_msgs cv_bridge std_msgs)

target_include_directories(hikvision_camera_node PRIVATE ${CURL_INCLUDE_DIRS})
target_link_libraries(hikvision_camera_node
  ${HIKVISION_LIB_DIR}/libhcnetsdk.so
  ${HIKVISION_LIB_DIR}/libhpr.so
  ${HIKVISION_LIB_DIR}/libHCCore.so
  ${HIKVISION_LIB_DIR}/libcrypto.so.1.1
  ${HIKVISION_LIB_DIR}/libssl.so.1.1
  ${CURL_LIBRARIES}
  ${OpenCV_LIBS}
)

# RPATH so executable finds SDK libs when run from install
set_target_properties(hikvision_camera_node PROPERTIES
  INSTALL_RPATH "${HIKVISION_LIB_DIR};${HIKVISION_LIB_DIR}/HCNetSDKCom"
  BUILD_WITH_INSTALL_RPATH TRUE
  INSTALL_RPATH_USE_LINK_PATH TRUE
)

install(TARGETS
  hikvision_camera_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
