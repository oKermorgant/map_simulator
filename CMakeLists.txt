cmake_minimum_required(VERSION 3.5)
project(map_simulator)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_BUILD_TYPE RELEASE)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PkgConfig)

rosidl_generate_interfaces(${PROJECT_NAME} "srv/Spawn.srv")

pkg_check_modules(YamlCpp yaml-cpp)

add_library(simulator_node SHARED src/simulator.cpp src/robot.cpp src/occupancy_grid.cpp)
target_include_directories(simulator_node PRIVATE include
                           ${OpenCV_INCLUDE_DIRS}
                           ${YamlCpp_INCLUDE_DIRS})
target_link_libraries(simulator_node ${OpenCV_LIBRARIES} yaml-cpp stdc++fs)
ament_target_dependencies(simulator_node rclcpp sensor_msgs geometry_msgs nav_msgs tf2_ros urdf)
rosidl_target_interfaces(simulator_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(spawn src/spawn.cpp)
ament_target_dependencies(spawn rclcpp)
rosidl_target_interfaces(spawn ${PROJECT_NAME} "rosidl_typesupport_cpp")

if(NOT WIN32)
  ament_environment_hooks(
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}"
    )
endif()

# auto-executable from node
rclcpp_components_register_node(simulator_node
  PLUGIN "map_simulator::SimulatorNode"
  EXECUTABLE simulator)

install(TARGETS simulator_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
  
install(TARGETS spawn DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch example
  DESTINATION share/${PROJECT_NAME}) 
  
ament_export_libraries(simulator_node)
ament_package()
