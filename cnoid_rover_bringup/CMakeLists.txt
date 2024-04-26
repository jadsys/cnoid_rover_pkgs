cmake_minimum_required(VERSION 3.5.0)
project(cnoid_rover_bringup)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  trajectory_msgs
  choreonoid
  )

  catkin_package(SKIP_CMAKE_CONFIG_GENERATION SKIP_PKG_CONFIG_GENERATION)
  
install(DIRECTORY
  model
  DESTINATION ${CHOREONOID_SHARE_SUBDIR}/cnoid_rover_bringup)
  
#install(DIRECTORY launch project
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)
