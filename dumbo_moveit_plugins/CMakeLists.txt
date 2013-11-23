cmake_minimum_required(VERSION 2.8.3)
project(dumbo_moveit_plugins)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Eigen REQUIRED)
find_package(Boost REQUIRED thread)

find_package(catkin COMPONENTS
  moveit_core
  pluginlib
  actionlib
  roscpp
  rosconsole
  urdf
  kdl_parser
  tf
  tf_conversions
  eigen_conversions
  control_msgs
  REQUIRED)

catkin_package(
  LIBRARIES
  INCLUDE_DIRS
  DEPENDS
    moveit_core
    )

include_directories(dumbo_arm_kinematics/include)


include_directories(SYSTEM ${EIGEN_INCLUDE_DIRS} ${Boost_INCLUDE_DIR})
include_directories(${catkin_INCLUDE_DIRS})

link_directories(${Boost_LIBRARY_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_subdirectory(dumbo_arm_kinematics)

install(
  FILES
    dumbo_moveit_kinematics_plugin_description.xml
  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION}
    )

