cmake_minimum_required(VERSION 3.15)
project(map_simulator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(TinyXML2 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

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
    rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
endif()

# get ROSDISTRO to adapt fors header extension
if("$ENV{ROS_DISTRO}" STRGREATER_EQUAL "lyrical")
  add_compile_definitions(ROS_HEADERS_H_DEPRECATED)
endif()

ament_auto_add_executable(simulator src/simulator.cpp src/robot.cpp src/occupancy_grid.cpp)
target_include_directories(simulator PRIVATE include
                           ${OpenCV_INCLUDE_DIRS}
                           ${YamlCpp_INCLUDE_DIRS})
target_link_libraries(simulator ${OpenCV_LIBRARIES} yaml-cpp stdc++fs tinyxml2)
if(${LEGACY_IDL})
    rosidl_target_interfaces(simulator ${PROJECT_NAME} "rosidl_typesupport_cpp")
else()
    target_link_libraries(simulator "${cpp_typesupport_target}")
endif()

ament_auto_add_executable(spawn src/spawn.cpp)
if(${LEGACY_IDL})
    rosidl_target_interfaces(spawn ${PROJECT_NAME} "rosidl_typesupport_cpp")
else()
    target_link_libraries(spawn "${cpp_typesupport_target}")
endif()

install(DIRECTORY launch example
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS src/kinematics.py
    DESTINATION lib/${PROJECT_NAME})

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
