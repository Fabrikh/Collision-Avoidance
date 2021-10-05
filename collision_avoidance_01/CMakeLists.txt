cmake_minimum_required(VERSION 3.0.2)
project(collision_avoidance_01)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs sensor_msgs message_generation)

generate_messages(DEPENDENCIES std_msgs geometry_msgs sensor_msgs)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(collision_avoidance src/collision_avoidance.cpp)
target_link_libraries(collision_avoidance ${catkin_LIBRARIES})
add_dependencies(collision_avoidance collision_avoidance_01_generate_messages_cpp)