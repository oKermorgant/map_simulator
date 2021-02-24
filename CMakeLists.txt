cmake_minimum_required(VERSION 2.8.3)
project(map_simulator)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
cmake_modules
roscpp
nav_msgs
sensor_msgs
tf2_ros
urdf
message_generation
)

find_package(OpenCV REQUIRED)
find_package(PkgConfig)

pkg_check_modules(YamlCpp yaml-cpp)

add_service_files(
   FILES
   Spawn.srv
 )

generate_messages(DEPENDENCIES std_msgs)

catkin_package()

add_executable(simulator src/simulator_node.cpp 
src/simulator.cpp
src/robot.cpp src/occupancy_grid.cpp
)
target_include_directories(simulator PRIVATE include
                           ${OpenCV_INCLUDE_DIRS}
                           ${YamlCpp_INCLUDE_DIRS}
                           ${catkin_INCLUDE_DIRS})
target_link_libraries(simulator ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} yaml-cpp stdc++fs)
add_dependencies(simulator map_simulator_gencpp)

add_executable(spawn src/spawn.cpp)
target_include_directories(spawn PRIVATE include ${catkin_INCLUDE_DIRS})
target_link_libraries(spawn ${catkin_LIBRARIES})
add_dependencies(spawn map_simulator_gencpp)

install(TARGETS simulator spawn
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  
install(DIRECTORY launch example
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}) 
install(FILES package.xml 
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
