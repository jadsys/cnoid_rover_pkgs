cmake_minimum_required(VERSION 3.5)
project(cnoid_rover_control)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING
    "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
    FORCE)
endif()

set(CHOREONOID_SKIP_QT_CONFIG true)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  trajectory_msgs
  nav_msgs
  tf
  choreonoid
  )

  catkin_package(
    CATKIN_DEPENDS roscpp std_msgs sensor_msgs trajectory_msgs nav_msgs tf
  )
  
if(CHOREONOID_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD ${CHOREONOID_CXX_STANDARD})
  set(CMAKE_CXX_EXTENSIONS OFF)
else()
  add_compile_options(-std=c++14)
endif()

include_directories(
  ${catkin_INCLUDE_DIRS}  include
  ${CHOREONOID_INCLUDE_DIRS}
  )

link_directories(
  ${CHOREONOID_LIBRARY_DIRS}
  )

add_subdirectory(src)
