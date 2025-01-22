cmake_minimum_required(VERSION 3.5)
project(audio_utils)


# Default to C99
if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(MUSIC_BEAT_DETECTOR_ENABLE_TESTS OFF CACHE BOOL "")

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(audio_utils_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

## System dependencies are found with CMake's conventions

# ******* ONNX Runtime dependency *******
if (EXISTS "/etc/nv_tegra_release")
    if (NOT EXISTS "/usr/local/lib/libonnxruntime.so")
        message(FATAL_ERROR "The ONNX Runtime is not installed.")
    endif()
    include_directories(SYSTEM /usr/local/include/onnxruntime/core/session)
    set(ONNXRUNTIME_LIBRARIES "/usr/local/lib/libonnxruntime.so")
elseif(${CMAKE_HOST_SYSTEM_PROCESSOR} MATCHES "x86_64")
    set(ONNXRUNTIME_NAME "onnxruntime-linux-x64-1.15.1")
elseif(${CMAKE_HOST_SYSTEM_PROCESSOR} MATCHES "aarch64")
    set(ONNXRUNTIME_NAME "onnxruntime-linux-aarch64-1.15.1")
else()
    message(FATAL_ERROR "The CPU architecture is not supported.")
endif()

if (ONNXRUNTIME_NAME)
    if (NOT EXISTS ${CMAKE_BINARY_DIR}/${ONNXRUNTIME_NAME}.tgz)
        file(DOWNLOAD https://github.com/microsoft/onnxruntime/releases/download/v1.15.1/${ONNXRUNTIME_NAME}.tgz ${CMAKE_BINARY_DIR}/${ONNXRUNTIME_NAME}.tgz SHOW_PROGRESS)
    endif()
    if (NOT EXISTS ${CMAKE_BINARY_DIR}/${ONNXRUNTIME_NAME}/VERSION_NUMBER)
        execute_process(COMMAND tar -xvf ${ONNXRUNTIME_NAME}.tgz WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
    endif()
    include_directories(SYSTEM ${CMAKE_BINARY_DIR}/${ONNXRUNTIME_NAME}/include)
    set(ONNXRUNTIME_LIBRARIES ${CMAKE_BINARY_DIR}/${ONNXRUNTIME_NAME}/lib/libonnxruntime.so)

    install(DIRECTORY ${CMAKE_BINARY_DIR}/${ONNXRUNTIME_NAME}/lib/ DESTINATION lib)
endif()

add_subdirectory(src/MusicBeatDetector)

include_directories(SYSTEM
    src/MusicBeatDetector/MusicBeatDetector/include
    src/MusicBeatDetector/3rdParty/fftw/build/include
    src/MusicBeatDetector/3rdParty/armadillo-code/include
    src/MusicBeatDetector/3rdParty/OpenBLAS
)

install(DIRECTORY src/MusicBeatDetector/MusicBeatDetector/include/ DESTINATION include)
install(DIRECTORY src/MusicBeatDetector/3rdParty/fftw/build/include/ DESTINATION include)
install(DIRECTORY src/MusicBeatDetector/3rdParty/armadillo-code/include/ DESTINATION include)
install(DIRECTORY src/MusicBeatDetector/3rdParty/OpenBLAS/ DESTINATION include)
install(TARGETS MusicBeatDetector DESTINATION lib)

# C++ Nodes
add_executable(beat_detector_node src/beat_detector_node.cpp)
ament_target_dependencies(beat_detector_node rclcpp std_msgs audio_utils_msgs)
target_link_libraries(beat_detector_node MusicBeatDetector)
install(TARGETS beat_detector_node DESTINATION lib/${PROJECT_NAME})


add_executable(capture_node src/capture_node.cpp
    src/PcmDevices/PcmDevice.cpp
    src/PcmDevices/AlsaPcmDevice.cpp
    src/PcmDevices/PulseAudioPcmDevice.cpp
    src/Utils/AlsaException.cpp
    src/Utils/PulseAudioException.cpp
)
ament_target_dependencies(capture_node rclcpp std_msgs audio_utils_msgs)
target_link_libraries(capture_node MusicBeatDetector asound pulse-simple pulse)
install(TARGETS capture_node DESTINATION lib/${PROJECT_NAME})


add_executable(playback_node src/playback_node.cpp
    src/PcmDevices/PcmDevice.cpp
    src/PcmDevices/AlsaPcmDevice.cpp
    src/PcmDevices/PulseAudioPcmDevice.cpp
    src/Utils/AlsaException.cpp
    src/Utils/PulseAudioException.cpp
    src/Utils/Semaphore.cpp
)
ament_target_dependencies(playback_node rclcpp std_msgs audio_utils_msgs)
target_link_libraries(playback_node MusicBeatDetector asound pulse-simple pulse)
install(TARGETS playback_node DESTINATION lib/${PROJECT_NAME})


add_executable(vad_node src/vad_node.cpp)
ament_target_dependencies(vad_node rclcpp std_msgs audio_utils_msgs ament_index_cpp)
target_link_libraries(vad_node MusicBeatDetector ${ONNXRUNTIME_LIBRARIES})
install(TARGETS vad_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY models DESTINATION share/${PROJECT_NAME})


# Python Nodes
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
    scripts/format_conversion_node.py
    scripts/raw_file_writer_node.py
    scripts/resampling_node.py
    scripts/split_channel_node.py
    DESTINATION lib/${PROJECT_NAME}
)

# Launch files
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# Exported libs
ament_export_include_directories(
    include
    ${CMAKE_CURRENT_SOURCE_DIR}/src/MusicBeatDetector/MusicBeatDetector/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src/MusicBeatDetector/3rdParty/fftw/build/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src/MusicBeatDetector/3rdParty/armadillo-code/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src/MusicBeatDetector/3rdParty/OpenBLAS
)
ament_export_libraries(
  MusicBeatDetector
  MusicBeatDetector_openblas
  MusicBeatDetector_fftw
)

ament_export_dependencies(audio_utils_msgs)

ament_package()
