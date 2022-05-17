cmake_minimum_required(VERSION 2.8.3)
project(mocap)
set(PROJECT_NAME mocap)
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

find_package(catkin REQUIRED)
find_package(Eigen3 REQUIRED)

## Find catkin macros and libraries
# list here the project dependencies, 3rd party packages such as rosbag or std_msgs
set(${PROJECT_NAME}_CATKIN_DEPS
    roscpp
    rospy
    roslib
    std_msgs
    gazebo_msgs
)

# find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    ${${PROJECT_NAME}_CATKIN_DEPS}
)

## Catkin specific configuration ##
catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPS} # message_runtime
)

#############
# LIBRARIES #
#############

include_directories(${PROJECT_NAME}_LIBRARY PRIVATE
  ${catkin_INCLUDE_DIRS}

  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/include

)

############
# BINARIES #
############

add_executable(mocap_bridge_gazebo src/mocap_bridge_gazebo.cpp)
target_link_libraries(mocap_bridge_gazebo ${catkin_LIBRARIES})

