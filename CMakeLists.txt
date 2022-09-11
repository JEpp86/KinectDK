cmake_minimum_required(VERSION 3.9.0)
project(kinect)

set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

#add_subdirectory(Microsoft/Azure-Kinect-Sensor-SDK)

include_directories(include)
include_directories(ThirdParty/Microsoft/include)

add_executable(read_cal app/read_cal_data.cpp src/Kinect.cpp)
target_link_libraries(read_cal PRIVATE k4a )

add_executable(capture_data app/capture_sensor_data.cpp src/Kinect.cpp)
target_link_libraries(capture_data PRIVATE k4a )

#add_executable(hello app/Hello_world.cpp)
#Microsoft/Azure-Kinect-Sensor-SDK/build/bin/Debug/