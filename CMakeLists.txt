# Enable C++11
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
cmake_minimum_required(VERSION 3.19)
# Define project name
project(AcquiringImagesMKV)

# Kinect DK相机
find_package(k4a REQUIRED)# 后面的target_link_libraries中用到了k4a::k4a
# Find OpenCV
find_package(OpenCV REQUIRED)

# Add includes
include_directories( ${OpenCV_INCLUDE_DIRS} )

# Declare the executable target built from your sources
add_executable(AcquiringImagesMKV main4.cpp)

# Link your application with other libraries
target_link_libraries(AcquiringImagesMKV k4a::k4a k4arecord ${OpenCV_LIBS})
