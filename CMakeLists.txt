cmake_minimum_required(VERSION 2.8.3)
project(simulation_2d)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
cmake_modules
roscpp
nav_msgs
sensor_msgs
tf2_ros
urdf
)

find_package(OpenCV REQUIRED)
find_package(PkgConfig)

pkg_check_modules(YamlCpp yaml-cpp)

catkin_package()


add_executable(simulator_2d src/simulator_2d.cpp src/robot2d.cpp src/occupancy_grid.cpp)
target_include_directories(simulator_2d PRIVATE include
                           ${catkin_INCLUDE_DIRS}
                           ${OpenCV_INCLUDE_DIRS}
                           ${YamlCpp_INCLUDE_DIRS})
target_link_libraries(simulator_2d
                      ${catkin_LIBRARIES}
                      ${OpenCV_LIBRARIES}
                      yaml-cpp
                      stdc++fs)
