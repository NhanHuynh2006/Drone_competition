cmake_minimum_required(VERSION 3.16)
project(drone_competition LANGUAGES C CXX)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# RPi5 Cortex-A76 optimization
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    set(RPI5 "-march=armv8.2-a+fp16+dotprod -mcpu=cortex-a76")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${RPI5}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${RPI5}")
endif()
set(CMAKE_C_FLAGS_RELEASE "-O3 -DNDEBUG")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")

find_package(OpenCV REQUIRED)
find_package(ncnn QUIET)
if(NOT ncnn_FOUND)
    set(NCNN_INC "/usr/local/include/ncnn" CACHE PATH "ncnn headers")
    set(NCNN_LIB "/usr/local/lib/libncnn.a" CACHE FILEPATH "ncnn lib")
    add_library(ncnn STATIC IMPORTED)
    set_target_properties(ncnn PROPERTIES IMPORTED_LOCATION ${NCNN_LIB}
                          INTERFACE_INCLUDE_DIRECTORIES ${NCNN_INC})
endif()

set(MAVLINK_DIR "${CMAKE_SOURCE_DIR}/deps")
find_package(Threads REQUIRED)
find_package(OpenMP QUIET)

# Include paths
include_directories(include ${MAVLINK_DIR} ${OpenCV_INCLUDE_DIRS})

# Sources
set(C_SRCS src/pid.c src/kalman2d.c src/dwm1001.c src/ball_dropper.c
           src/visual_servo.c src/mission.c src/flight_controller.c)
set(CXX_SRCS src/detector.cpp src/hsv_detector.cpp src/sensor_fusion.cpp src/main.cpp)

add_executable(${PROJECT_NAME} ${C_SRCS} ${CXX_SRCS})
target_link_libraries(${PROJECT_NAME} PRIVATE ${OpenCV_LIBS} ncnn Threads::Threads m
    $<$<BOOL:${OpenMP_CXX_FOUND}>:OpenMP::OpenMP_CXX> gomp)

# HSV-only test (no NCNN needed) - disabled: hsv_detector.cpp has no standalone main
# add_executable(test_hsv src/hsv_detector.cpp)
# target_compile_definitions(test_hsv PRIVATE HSV_STANDALONE)
# target_include_directories(test_hsv PRIVATE include ${OpenCV_INCLUDE_DIRS})
# target_link_libraries(test_hsv PRIVATE ${OpenCV_LIBS})
