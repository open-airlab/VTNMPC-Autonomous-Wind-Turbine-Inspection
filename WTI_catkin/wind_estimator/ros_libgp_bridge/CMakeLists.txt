cmake_minimum_required(VERSION 2.8.3)
project(ros_libgp_bridge)

SET(LIBGP_LIB_PATH ${PROJECT_SOURCE_DIR}/libgp)
ADD_SUBDIRECTORY(libgp)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX14)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX14)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
elseif(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++14 or C++11 support. Please use a different C++ compiler.")
endif()


find_package(catkin REQUIRED COMPONENTS
  roscpp
)

#find_package(Eigen3 3.0.1 REQUIRED)
#include_directories(${EIGEN3_INCLUDE_DIRS})

catkin_package(
	#INCLUDE_DIRS_libgp libgp/src
	#INCLUDE_DIRS_toolkit libgp

	INCLUDE_DIRS libgp
)







