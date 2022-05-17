cmake_minimum_required(VERSION 2.8.3)
project(acado_ccode_generation)
set(PROJECT_NAME acado_ccode_generation)
set(ROS_BUILD_TYPE Debug)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX14)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX14)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
elseif(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 ${CMAKE_CXX_FLAGS_DEBUG} -O1")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++14 or C++11 support. Please use a different C++ compiler.")
endif()

## Custom options
set(MAKE_SHARED true CACHE BOOL "Build and use shared libraries")
## Define settings for static build
if(NOT MAKE_SHARED)
  ## This is needed by find_package
  set(CMAKE_FIND_LIBRARY_SUFFIXES ".a")
  ## Link against static libs
  set(CMAKE_EXE_LINKER_FLAGS "-static-libgcc -static-libstdc++ -static")
  ## Produce static libs
  set(BUILD_SHARED_LIBS OFF)
endif()

## Find catkin macros and libraries
# list here the project dependencies, 3rd party packages
# such as rosbag or std_msgs
set(${PROJECT_NAME}_CATKIN_DEPS
    roscpp 
    rospy 
    roslib 
    ros_acado_bridge
)


# find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    ${${PROJECT_NAME}_CATKIN_DEPS}
)


find_package(Eigen3 3.0.1 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPS}
)

#############
# LIBRARIES #
#############

set(PROJECT_LIBS
    ${catkin_LIBRARIES}
    # other
    acado_toolkit_s
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/include

  ${ros_acado_bridge_INCLUDE_DIRS}
  ${ros_acado_bridge_INCLUDE_DIRS}/acado
)

############
# BINARIES #
############
   
option(QUADROTOR "flag to compile quadrotor codes" ON)
if(QUADROTOR)
    # NMPC C code generation
    add_executable(nmpc_pc_learning_ccode 
	   src/quadrotor/nmpc_pc_learning_ccode.cpp
	)
	target_link_libraries(nmpc_pc_learning_ccode
      	   ${PROJECT_LIBS}
	)
	
	add_executable(posaware_ratecommand_nmpc_pc_learning_ccode 
	   src/quadrotor/posaware_ratecommand_nmpc_pc_learning_ccode.cpp
	)
	target_link_libraries(posaware_ratecommand_nmpc_pc_learning_ccode
      	   ${PROJECT_LIBS}
	)

	add_executable(posaware_ratecommand_quaternion_nmpc_pc_learning_ccode 
	   src/quadrotor/posaware_ratecommand_quaternion_nmpc_pc_learning_ccode.cpp
	)
	target_link_libraries(posaware_ratecommand_quaternion_nmpc_pc_learning_ccode
      	   ${PROJECT_LIBS}
	)
	
	add_executable(nmpc_pc_learning_posaware_ccode 
	   src/quadrotor/nmpc_pc_learning_posaware_ccode.cpp
	)
	target_link_libraries(nmpc_pc_learning_posaware_ccode
      	   ${PROJECT_LIBS}
	)

	add_executable(PAMPC_ccode 
	   src/quadrotor/PAMPC_ccode.cpp
	)
	target_link_libraries(PAMPC_ccode
      	   ${PROJECT_LIBS}
	)

	add_executable(nmpc_pc_quadrotor_ccode 
	   src/quadrotor/nmpc_pc_quadrotor_ccode.cpp
	)
	target_link_libraries(nmpc_pc_quadrotor_ccode
      	   ${PROJECT_LIBS}
	)
	add_executable(quaternion_point_traj_ccode 
	   src/quadrotor/quaternion_point_traj_ccode.cpp
	)
	target_link_libraries(quaternion_point_traj_ccode
      	   ${PROJECT_LIBS}
	)


endif(QUADROTOR)

