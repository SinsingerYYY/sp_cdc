cmake_minimum_required(VERSION 3.16)

find_package(yaml-cpp REQUIRED)

add_subdirectory(serial)

# 创建目标 io
add_library(io STATIC 
    hikrobot/hikrobot.cpp    
    mindvision/mindvision.cpp  
    usbcamera/usbcamera.cpp  
    camera.cpp
    cboard.cpp
    dm_imu/dm_imu.cpp
    gimbal/gimbal.cpp
    mavlink/mavlink_io.cpp
    minipc/minipc_io.cpp
)

# hikrobot
# 优先使用系统安装的完整 MVS SDK（含 USB3 传输层），否则回退到项目自带 SDK
set(HIKROBOT_SDK_ROOT "/opt/MVS" CACHE PATH "HikRobot MVS SDK root")
if(EXISTS "${HIKROBOT_SDK_ROOT}/include/MvCameraControl.h")
  message(STATUS "Using system HikRobot MVS SDK: ${HIKROBOT_SDK_ROOT}")
  target_include_directories(io PUBLIC "${HIKROBOT_SDK_ROOT}/include")
  target_link_directories(io PUBLIC "${HIKROBOT_SDK_ROOT}/lib/64")
else()
  message(STATUS "Using bundled HikRobot SDK under io/hikrobot")
  target_include_directories(io PUBLIC hikrobot/include)
  if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
    target_link_directories(io PUBLIC hikrobot/lib/amd64)
  elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    target_link_directories(io PUBLIC hikrobot/lib/arm64)
  else()
    message(FATAL_ERROR "Unsupported architecture: ${CMAKE_HOST_SYSTEM_PROCESSOR}!")
  endif()
endif()

# mindvision
target_include_directories(io PUBLIC mindvision/include)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  target_link_directories(io PUBLIC mindvision/lib/amd64)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  target_link_directories(io PUBLIC mindvision/lib/arm64)
else()
  message(FATAL_ERROR "Unsupported architecture: ${CMAKE_HOST_SYSTEM_PROCESSOR}!")
endif()

target_link_libraries(io MvCameraControl MVSDK usb-1.0 yaml-cpp serial)

################## 检查 ROS 环境 ##################
find_package(ament_cmake QUIET)
find_package(rclcpp QUIET)
find_package(std_msgs QUIET)
find_package(rosidl_typesupport_cpp QUIET)
find_package(sp_msgs QUIET)

# 如果找不到 ROS 环境，则跳过 ROS 相关的部分
if(NOT ament_cmake_FOUND OR NOT rclcpp_FOUND OR NOT std_msgs_FOUND OR NOT rosidl_typesupport_cpp_FOUND OR NOT sp_msgs_FOUND)
    message(WARNING "ROS2 not found, skipping ROS2 specific code.")
else()
    message(STATUS "ROS2 environment found, compiling ROS2-related code.")
    
    # 添加 ROS2 相关的文件
    target_sources(io PRIVATE ros2/publish2nav.cpp ros2/ros2.cpp ros2/subscribe2nav.cpp)

    # 设置 ROS2 依赖
    include_directories(${std_msgs_INCLUDE_DIRS})
    ament_target_dependencies(io rclcpp std_msgs sp_msgs)
endif()
