cmake_minimum_required(VERSION 2.8.3)
project(boxbot_finds_table)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  actionlib
  actionlib_msgs
)

find_package(Boost REQUIRED COMPONENTS system)

add_action_files(
  DIRECTORY action
  FILES Fibonacci.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
)

catkin_package(
  INCLUDE_DIRS src
  LIBRARIES boxbot_finds_table
  CATKIN_DEPENDS roscpp tf actionlib_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

# Link the nodes
add_library(table_finder_algo src/table_finder_algo.cpp)
add_dependencies(
  table_finder_algo
  ${catkin_EXPORTED_TARGETS}
  ${boxbot_finds_table_EXPORTED_TARGETS}
)


add_executable(table_finder src/table_finder_node.cpp)
add_dependencies(
  table_finder
  ${catkin_EXPORTED_TARGETS}
  ${boxbot_finds_table_EXPORTED_TARGETS}
)

target_link_libraries(
  table_finder
  table_finder_algo
  ${catkin_LIBRARIES}
)

add_executable(fibonacci_client src/fibonacci_client.cpp)

target_link_libraries(
  fibonacci_client
  ${catkin_LIBRARIES}
)

add_dependencies(
  fibonacci_client
  ${boxbot_finds_table_EXPORTED_TARGETS}
)
