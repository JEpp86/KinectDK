cmake_minimum_required(VERSION 3.9.0)
project(kinect)

set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

find_package(OpenCV CONFIG REQUIRED PATHS "../opencv/build/install")
message(STATUS "OpenCV directories: ${OpenCV_INCLUDE_DIRS}")

#add_subdirectory(Microsoft/Azure-Kinect-Sensor-SDK)

include_directories(include)
include_directories(ThirdParty/Microsoft/include)
include_directories("${OpenCV_INCLUDE_DIRS}")

add_library(kinect src/Kinect.cpp src/KinectUtils.cpp)
target_link_libraries(kinect k4a k4arecord ${OpenCV_LIBS})

add_executable(read_cal app/read_cal_data.cpp) # src/Kinect.cpp)
target_link_libraries(read_cal PRIVATE kinect k4a )

add_executable(capture_data app/capture_sensor_data.cpp) # src/Kinect.cpp)
target_link_libraries(capture_data PRIVATE kinect k4a ${OpenCV_LIBS})

add_executable(process_record app/process_record.cpp) # src/Kinect.cpp)
target_link_libraries(process_record PRIVATE kinect k4a k4arecord ${OpenCV_LIBS})
