set(MOVEIT_LIB_NAME dumbo_moveit_arm_kinematics)

add_library(${MOVEIT_LIB_NAME} 
  src/dumbo_arm_ik.cpp
  src/dumbo_arm_kinematics_utils.cpp
  src/dumbo_arm_ik_solver.cpp
  src/dumbo_arm_kinematics_plugin.cpp
  src/dumbo_kinematics.c
  src/homogenic_vectors.c
  )

target_link_libraries(${MOVEIT_LIB_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS ${MOVEIT_LIB_NAME} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})