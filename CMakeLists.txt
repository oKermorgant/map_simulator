cmake_minimum_required(VERSION 3.5)
project(simulation_2d)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(OpenCV REQUIRED)
find_package(PkgConfig)

pkg_check_modules(YamlCpp yaml-cpp)
add_executable(simulator_2d src/simulator_2d.cpp src/robot2d.cpp src/occupancy_grid.cpp)
target_include_directories(simulator_2d PRIVATE include
                           ${catkin_INCLUDE_DIRS}
                           ${OpenCV_INCLUDE_DIRS}
                           ${YamlCpp_INCLUDE_DIRS})
target_link_libraries(simulator_2d ${OpenCV_LIBRARIES} yaml-cpp stdc++fs)
ament_target_dependencies(simulator_2d rclcpp sensor_msgs geometry_msgs nav_msgs urdf tf2_ros)

install(TARGETS simulator_2d
  DESTINATION lib/${PROJECT_NAME})

ament_package()
