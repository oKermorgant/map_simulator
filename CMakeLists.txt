cmake_minimum_required(VERSION 3.5)
project(map_simulator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(tinyxml2_vendor REQUIRED)
find_package(TinyXML2 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)

set(CMAKE_CXX_STANDARD 17)

if("$ENV{ROS_DISTRO}" STRLESS_EQUAL "humble")
    add_definitions(-DWITH_BUILTIN_RANGE)
    rosidl_generate_interfaces(${PROJECT_NAME} "srv/Spawn.srv" "srv/AddAnchor.srv" "msg/Range.msg"
                            DEPENDENCIES std_msgs)
else()
    rosidl_generate_interfaces(${PROJECT_NAME} "srv/Spawn.srv" "srv/AddAnchor.srv")
endif()

if("$ENV{ROS_DISTRO}" STRLESS_EQUAL "galactic")
    set(LEGACY_IDL TRUE)
else()
    set(LEGACY_IDL FALSE)
endif()
if(NOT ${LEGACY_IDL})
    rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
endif()

add_executable(simulator src/simulator.cpp src/robot.cpp src/occupancy_grid.cpp)
target_include_directories(simulator PRIVATE include
                           ${OpenCV_INCLUDE_DIRS}
                           ${YamlCpp_INCLUDE_DIRS})
target_link_libraries(simulator ${OpenCV_LIBRARIES} yaml-cpp stdc++fs tinyxml2)
ament_target_dependencies(simulator rclcpp sensor_msgs geometry_msgs nav_msgs tf2_ros urdf)
if(${LEGACY_IDL})
    rosidl_target_interfaces(simulator ${PROJECT_NAME} "rosidl_typesupport_cpp")
else()
    target_link_libraries(simulator "${cpp_typesupport_target}")
endif()

add_executable(spawn src/spawn.cpp)
ament_target_dependencies(spawn rclcpp)
if(${LEGACY_IDL})
    rosidl_target_interfaces(spawn ${PROJECT_NAME} "rosidl_typesupport_cpp")
else()
    target_link_libraries(spawn "${cpp_typesupport_target}")
endif()

if(NOT WIN32)
  ament_environment_hooks(
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}"
    )
endif()

install(TARGETS simulator spawn
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch example
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS src/kinematics.py
    DESTINATION lib/${PROJECT_NAME})

#ament_export_libraries(simulator)
ament_package()
