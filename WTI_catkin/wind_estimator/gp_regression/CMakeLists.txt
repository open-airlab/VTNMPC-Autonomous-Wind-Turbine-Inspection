cmake_minimum_required(VERSION 2.8.3)
project(gp_regression)
set(PROJECT_NAME gp_regression)
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

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs roslib nav_msgs mavros_msgs tf ros_libgp_bridge
)


find_package(Eigen3 3.0.1 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp tf geometry_msgs mavros_msgs
)

#############
# LIBRARIES #
#############

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/include

  ${ros_libgp_bridge_INCLUDE_DIRS}
  ${ros_libgp_bridge_INCLUDE_DIRS}/include
  ${ros_libgp_bridge_INCLUDE_DIRS}/src
)

############
# BINARIES #
############

if(true)
	# libgp C code generation
#	add_executable(gp_disturb_main_combined 
#	   src/gp_disturb_main_combined.cc
#	   src/gp_disturb_reg.cc
#	)
#	target_link_libraries(gp_disturb_main_combined
#      	   ${catkin_LIBRARIES}
#	   gp
#	)
	add_executable(gp_disturb_main 
	   src/gp_disturb_main.cc
	   src/gp_disturb_reg.cc
	)
	target_link_libraries(gp_disturb_main
      	   ${catkin_LIBRARIES}
	   gp
	)
#	add_executable(gp_disturb_main_y 
#	   src/gp_disturb_main_y.cc
#	   src/gp_disturb_reg.cc
#	)
#	target_link_libraries(gp_disturb_main_y
#      	   ${catkin_LIBRARIES}
#	   gp
#	)
#	add_executable(gp_disturb_main_z 
#	   src/gp_disturb_main_z.cc
#	   src/gp_disturb_reg.cc
#	)
#	target_link_libraries(gp_disturb_main_z
#     	   ${catkin_LIBRARIES}
#	   gp
#	)
endif()
