cmake_minimum_required(VERSION 3.8)
project(hikvision_camera_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# SDK path: 1) package-local sdk/ (clone-and-build), 2) env -D, 3) parent ..
set(HIKVISION_SDK_ROOT "" CACHE PATH "Hikvision SDK root")
if(NOT HIKVISION_SDK_ROOT AND DEFINED ENV{HIKVISION_SDK_ROOT})
  set(HIKVISION_SDK_ROOT "$ENV{HIKVISION_SDK_ROOT}" CACHE PATH "" FORCE)
endif()
if(NOT HIKVISION_SDK_ROOT)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/sdk/incEn/HCNetSDK.h")
    set(HIKVISION_SDK_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/sdk" CACHE PATH "" FORCE)
  else()
    set(HIKVISION_SDK_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/.." CACHE PATH "" FORCE)
  endif()
endif()
set(HIKVISION_INCLUDE_DIR "${HIKVISION_SDK_ROOT}/incEn")
set(HIKVISION_LIB_DIR "${HIKVISION_SDK_ROOT}/lib")

if(NOT EXISTS "${HIKVISION_INCLUDE_DIR}/HCNetSDK.h")
  message(FATAL_ERROR
    "Hikvision SDK not found at: ${HIKVISION_SDK_ROOT}\n"
    "  Put sdk/ (incEn + lib) inside this package, or:\n"
    "  colcon build --cmake-args -DHIKVISION_SDK_ROOT=/path/to/sdk")
endif()
if(NOT EXISTS "${HIKVISION_LIB_DIR}/libhcnetsdk.so")
  message(FATAL_ERROR "Hikvision SDK lib not found at: ${HIKVISION_LIB_DIR}")
endif()
message(STATUS "Hikvision SDK: ${HIKVISION_SDK_ROOT}")

# Install SDK libs next to executables so run from install works on any PC
set(SDK_LIBS_INSTALL_DIR "lib/${PROJECT_NAME}/sdk_libs")
install(DIRECTORY ${HIKVISION_LIB_DIR}/
  DESTINATION ${SDK_LIBS_INSTALL_DIR}
  USE_SOURCE_PERMISSIONS
  FILES_MATCHING PATTERN "*.so" PATTERN "*.so.*")
install(DIRECTORY ${HIKVISION_LIB_DIR}/HCNetSDKCom/
  DESTINATION ${SDK_LIBS_INSTALL_DIR}/HCNetSDKCom
  USE_SOURCE_PERMISSIONS
  FILES_MATCHING PATTERN "*.so")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(CURL REQUIRED)
find_package(message_filters REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME} "msg/ImagePair.msg" DEPENDENCIES sensor_msgs)

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

# RPATH: installed executable finds SDK libs in sdk_libs/ next to it (relocatable)
set_target_properties(hikvision_camera_node PROPERTIES
  INSTALL_RPATH "\$ORIGIN/sdk_libs;\$ORIGIN/sdk_libs/HCNetSDKCom"
  BUILD_WITH_INSTALL_RPATH TRUE
  INSTALL_RPATH_USE_LINK_PATH FALSE
)

add_executable(image_rotate_node src/image_rotate_node.cpp)
ament_target_dependencies(image_rotate_node rclcpp sensor_msgs cv_bridge)
target_link_libraries(image_rotate_node ${OpenCV_LIBS})

add_executable(image_sync_node src/image_sync_node.cpp)
ament_target_dependencies(image_sync_node rclcpp sensor_msgs message_filters)
target_link_libraries(image_sync_node "${PROJECT_NAME}__rosidl_typesupport_cpp")
target_include_directories(image_sync_node PRIVATE
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/include)

install(TARGETS
  hikvision_camera_node
  image_rotate_node
  image_sync_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  msg
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
