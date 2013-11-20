cmake_minimum_required(VERSION 2.8.3)
project(dumbo_moveit_examples)


find_package(catkin REQUIRED COMPONENTS roscpp moveit_core moveit_ros_planning moveit_ros_planning_interface)
find_package(Eigen REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)

catkin_package(
  # DEPENDS
  CATKIN_DEPENDS roscpp moveit_core moveit_ros_planning moveit_ros_planning_interface
  # INCLUDE 
  # LIBRARIES 
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)

link_directories(${catkin_LIBRARY_DIRS})

add_executable(dumbo_move_group_example src/dumbo_move_group_example.cpp)
target_link_libraries(dumbo_move_group_example ${catkin_LIBRARIES} ${Boost_LIBRARIES})

