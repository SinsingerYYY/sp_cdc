cmake_minimum_required(VERSION 3.16.3)

project(sp_vision)

include(CTest)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)
message(STATUS "--------------------CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}--------------------")

find_package(OpenCV REQUIRED)
find_package(fmt REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(spdlog REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(nlohmann_json REQUIRED)
if(NOT OpenVINO_DIR)
    set(OpenVINO_DIR "/opt/intel/openvino_2024.4.0/runtime/cmake/")
endif()
find_package(OpenVINO REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR})


add_subdirectory(tools)
add_subdirectory(io)
add_subdirectory(tasks/auto_aim)
add_subdirectory(tasks/auto_buff)
add_subdirectory(tasks/omniperception)

####################src###################
add_executable(standard src/standard.cpp)
add_executable(mt_standard src/mt_standard.cpp)
add_executable(standard_mpc src/standard_mpc.cpp)
add_executable(auto_aim_debug_mpc src/auto_aim_debug_mpc.cpp)
add_executable(mt_auto_aim_debug src/mt_auto_aim_debug.cpp)
add_executable(auto_buff_debug src/auto_buff_debug.cpp)
add_executable(auto_buff_debug_mpc src/auto_buff_debug_mpc.cpp)
add_executable(uav src/uav.cpp)
add_executable(uav_debug src/uav_debug.cpp)
add_executable(local_debug src/local_debug.cpp)
add_executable(sentry_mavlink src/sentry_mavlink.cpp)
add_executable(sentry_minipc src/sentry_minipc.cpp)

set(SP_DEPS ${OpenCV_LIBS} fmt::fmt yaml-cpp spdlog::spdlog)

target_link_libraries(standard auto_aim auto_buff tools io ${SP_DEPS})
target_link_libraries(mt_standard auto_aim auto_buff tools io ${SP_DEPS})
target_link_libraries(standard_mpc auto_aim auto_buff tools io ${SP_DEPS})
target_link_libraries(auto_aim_debug_mpc auto_aim auto_buff tools io ${SP_DEPS})
target_link_libraries(mt_auto_aim_debug auto_aim auto_buff tools io ${SP_DEPS})
target_link_libraries(auto_buff_debug auto_buff tools io ${SP_DEPS} ${CERES_LIBRARIES})
target_link_libraries(auto_buff_debug_mpc auto_buff tools io ${SP_DEPS} ${CERES_LIBRARIES})
target_link_libraries(uav auto_aim auto_buff tools io ${SP_DEPS})
target_link_libraries(uav_debug auto_aim tools io ${SP_DEPS})
target_link_libraries(local_debug auto_aim tools io ${SP_DEPS})
target_link_libraries(sentry_mavlink auto_aim tools io ${SP_DEPS})
target_link_libraries(sentry_minipc auto_aim tools io ${SP_DEPS})

##################calibration################
add_executable(capture calibration/capture.cpp)
add_executable(capture_minipc calibration/capture_minipc.cpp)
add_executable(record calibration/record.cpp)
add_executable(extract_frames calibration/extract_frames.cpp)
add_executable(calibrate_camera calibration/calibrate_camera.cpp)
add_executable(calibrate_handeye calibration/calibrate_handeye.cpp)
add_executable(calibrate_robotworld_handeye calibration/calibrate_robotworld_handeye.cpp)
add_executable(split_video calibration/split_video.cpp)

target_link_libraries(capture tools io ${SP_DEPS})
target_link_libraries(capture_minipc tools io ${SP_DEPS})
target_link_libraries(record tools io ${SP_DEPS})
target_link_libraries(extract_frames ${SP_DEPS})
target_link_libraries(calibrate_camera tools ${SP_DEPS})
target_link_libraries(calibrate_handeye tools ${SP_DEPS})
target_link_libraries(calibrate_robotworld_handeye tools ${SP_DEPS})
target_link_libraries(split_video tools ${SP_DEPS})

##################tests##################
add_executable(auto_aim_test tests/auto_aim_test.cpp)
add_executable(auto_buff_test tests/auto_buff_test.cpp)
add_executable(camera_detect_test tests/camera_detect_test.cpp)
add_executable(camera_test tests/camera_test.cpp)
add_executable(camera_thread_test tests/camera_thread_test.cpp)
add_executable(cboard_test tests/cboard_test.cpp)
add_executable(fire_test tests/fire_test.cpp)
add_executable(detector_video_test tests/detector_video_test.cpp)
add_executable(gimbal_response_test tests/gimbal_response_test.cpp)
add_executable(multi_usbcamera_test tests/multi_usbcamera_test.cpp)
add_executable(usbcamera_detect_test tests/usbcamera_detect_test.cpp)
add_executable(usbcamera_test tests/usbcamera_test.cpp)
add_executable(handeye_test tests/handeye_test.cpp)
add_executable(dm_test tests/dm_test.cpp)
add_executable(minimum_vision_system tests/minimum_vision_system.cpp)
add_executable(minipc_send_test tests/minipc_send_test.cpp)
add_executable(grab_frame tests/grab_frame.cpp)

target_link_libraries(minipc_send_test tools io ${SP_DEPS})
target_link_libraries(grab_frame tools io ${SP_DEPS})

target_link_libraries(auto_aim_test auto_aim tools io ${SP_DEPS})
target_link_libraries(auto_buff_test auto_buff tools io ${SP_DEPS})
target_link_libraries(camera_detect_test auto_aim tools io ${SP_DEPS})
target_link_libraries(camera_test tools io ${SP_DEPS})
target_link_libraries(camera_thread_test auto_aim tools io ${SP_DEPS})
target_link_libraries(cboard_test tools io ${SP_DEPS})
target_link_libraries(fire_test tools io ${SP_DEPS})
target_link_libraries(detector_video_test auto_aim tools ${SP_DEPS})
target_link_libraries(gimbal_response_test tools io ${SP_DEPS})
target_link_libraries(multi_usbcamera_test tools io ${SP_DEPS})
target_link_libraries(usbcamera_detect_test auto_aim tools io ${SP_DEPS})
target_link_libraries(usbcamera_test tools io ${SP_DEPS})
target_link_libraries(handeye_test auto_aim tools io ${SP_DEPS})
target_link_libraries(dm_test tools io ${SP_DEPS})
target_link_libraries(minimum_vision_system auto_aim tools io ${SP_DEPS})

add_executable(gimbal_test tests/gimbal_test.cpp)
target_link_libraries(gimbal_test auto_aim tools io ${SP_DEPS})

add_executable(planner_test tests/planner_test.cpp)
target_link_libraries(planner_test auto_aim tools io ${SP_DEPS})

add_executable(planner_test_offline tests/planner_test_offline.cpp)
target_link_libraries(planner_test_offline auto_aim tools io ${SP_DEPS})

# 检查 ROS 环境
find_package(ament_cmake QUIET)
find_package(rclcpp QUIET)
find_package(std_msgs QUIET)
find_package(rosidl_typesupport_cpp QUIET)
find_package(sp_msgs QUIET)
# 只有在找到 ROS 环境时才会添加 sentry 和 publish_test
if(ament_cmake_FOUND AND rclcpp_FOUND AND std_msgs_FOUND AND rosidl_typesupport_cpp_FOUND AND sp_msgs_FOUND)
    # 编译 sentry 和 publish_test
    add_executable(sentry src/sentry.cpp)
    add_executable(sentry_bp src/sentry_bp.cpp)
    add_executable(sentry_debug src/sentry_debug.cpp)
    add_executable(sentry_multithread src/sentry_multithread.cpp)
    add_executable(publish_test tests/publish_test.cpp)
    add_executable(subscribe_test tests/subscribe_test.cpp)
    add_executable(topic_loop_test tests/topic_loop_test.cpp)

    target_link_libraries(sentry auto_aim omniperception tools io ${SP_DEPS})
    target_link_libraries(sentry_bp auto_aim omniperception tools io ${SP_DEPS})
    target_link_libraries(sentry_debug auto_aim omniperception tools io ${SP_DEPS})
    target_link_libraries(sentry_multithread auto_aim omniperception tools io ${SP_DEPS})
    target_link_libraries(publish_test auto_aim tools io ${SP_DEPS})
    target_link_libraries(subscribe_test tools io ${SP_DEPS})
    target_link_libraries(topic_loop_test tools io ${SP_DEPS})
else()
    message(STATUS "ROS2 environment not found, skipping ROS2-related code.")
endif()

##################ctest##################
# 离线单元测试：不依赖相机、串口和 ROS，可用 `ctest --output-on-failure` 运行
if(BUILD_TESTING)
    add_executable(aimer_spin_selection_test tests/aimer_spin_selection_test.cpp)
    target_compile_definitions(
        aimer_spin_selection_test PRIVATE
        AIMER_SPIN_SELECTION_TEST_CONFIG="${PROJECT_SOURCE_DIR}/tests/aimer_spin_selection_test.yaml"
        AIMER_SPIN_SELECTION_INVALID_CONFIG="${PROJECT_SOURCE_DIR}/tests/aimer_spin_selection_invalid.yaml")
    target_link_libraries(aimer_spin_selection_test auto_aim auto_buff tools io ${SP_DEPS})
    add_test(NAME aimer_spin_selection_test COMMAND aimer_spin_selection_test)

    add_executable(aimer_pitch_offset_test tests/aimer_pitch_offset_test.cpp)
    target_compile_definitions(
        aimer_pitch_offset_test PRIVATE
        AIMER_PITCH_OFFSET_TEST_CONFIG_DIR="${PROJECT_SOURCE_DIR}/tests")
    target_link_libraries(aimer_pitch_offset_test auto_aim auto_buff tools io ${SP_DEPS})
    add_test(NAME aimer_pitch_offset_test COMMAND aimer_pitch_offset_test)

    add_executable(target_layout_test tests/target_layout_test.cpp)
    target_compile_definitions(
        target_layout_test PRIVATE
        TARGET_LAYOUT_TEST_CONFIG="${PROJECT_SOURCE_DIR}/tests/target_layout_test.yaml")
    target_link_libraries(target_layout_test auto_aim auto_buff tools io ${SP_DEPS})
    add_test(NAME target_layout_test COMMAND target_layout_test)

    add_executable(outpost_model_test tests/outpost_model_test.cpp)
    target_compile_definitions(
        outpost_model_test PRIVATE
        OUTPOST_MODEL_TEST_CONFIG="${PROJECT_SOURCE_DIR}/tests/outpost_model_test.yaml")
    target_link_libraries(outpost_model_test auto_aim auto_buff tools io ${SP_DEPS})
    add_test(NAME outpost_model_test COMMAND outpost_model_test)

    add_executable(solver_image_size_test tests/solver_image_size_test.cpp)
    target_compile_definitions(
        solver_image_size_test PRIVATE
        SOLVER_IMAGE_SIZE_TEST_CONFIG="${PROJECT_SOURCE_DIR}/tests/solver_image_size.yaml")
    target_link_libraries(solver_image_size_test auto_aim auto_buff tools io ${SP_DEPS})
    add_test(NAME solver_image_size_test COMMAND solver_image_size_test)

    add_executable(command_safety_test tests/command_safety_test.cpp)
    target_include_directories(command_safety_test PRIVATE ${PROJECT_SOURCE_DIR})
    add_test(NAME command_safety_test COMMAND command_safety_test)

    add_executable(handeye_transform_test tests/handeye_transform_test.cpp)
    target_link_libraries(handeye_transform_test PRIVATE ${OpenCV_LIBS} Eigen3::Eigen)
    add_test(NAME handeye_transform_test COMMAND handeye_transform_test)
endif()
