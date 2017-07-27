cmake_minimum_required(VERSION 2.8.3)
project(behaviours)

SET(CMAKE_CXX_FLAGS "-std=c++11")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
  random_numbers
  tf
)

add_message_files(
    FILES
    PheromoneTrail.msg
    Rover.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs std_msgs random_numbers tf message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(
  behaviours
  src/ObstacleController.cpp 
  src/PickUpController.cpp
  src/DropOffController.cpp
  src/CPFASearchController.cpp
  src/ROSAdapter.cpp
  src/PID.cpp
  src/DriveController.cpp
  src/LogicController.cpp
  src/Pheromone.cpp
)

add_dependencies(behaviours ${catkin_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(
  behaviours
  ${catkin_LIBRARIES}
)

