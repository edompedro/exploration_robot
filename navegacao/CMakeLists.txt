cmake_minimum_required(VERSION 2.8.3)
project(navegacao)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp geometry_msgs sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(slam src/slam.cpp)
target_link_libraries(slam ${catkin_LIBRARIES})
add_executable(tele_operation src/tele_operation.cpp)
target_link_libraries(tele_operation ${catkin_LIBRARIES})
