cmake_minimum_required(VERSION 2.8.3)
project(mobility)

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
  mobility 
  src/PickUpController.cpp
  src/DropOffController.cpp
  src/SearchController.cpp
  src/mobility.cpp
  src/Pheromone.cpp
)

add_dependencies(mobility ${catkin_EXPORTED_TARGETS} ${mobility_EXPORTED_TARGETS})

target_link_libraries(
  mobility
  ${catkin_LIBRARIES}
)

