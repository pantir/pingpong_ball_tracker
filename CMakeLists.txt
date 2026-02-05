cmake_minimum_required(VERSION 4.2.0)
project(blackfly_tracker)

set(CMAKE_CXX_STANDARD 17)

# -----------------------
# Spinnaker
# -----------------------
set(SPINNAKER_INCLUDE_DIR "C:/Program Files/Teledyne/Spinnaker/include")
set(SPINNAKER_LIB_DIR "C:/Program Files/Teledyne/Spinnaker/lib64/vs2015")

include_directories(${SPINNAKER_INCLUDE_DIR})
link_directories(${SPINNAKER_LIB_DIR})

# Core Spinnaker library (required)
set(SPINNAKER_LIB Spinnaker_v140)

# OpenCV
set(OpenCV_DIR "C:/opencv/opencv/build/x64/vc16/lib")
find_package(OpenCV REQUIRED)

add_executable(blackfly src/main.cpp)
target_link_libraries(blackfly ${SPINNAKER_LIB} ${OpenCV_LIBS})