cmake_minimum_required(VERSION 2.8.3)
project(ros_acado_bridge)

SET(ACADO_LIB_PATH ${PROJECT_SOURCE_DIR}/ACADOtoolkit)
ADD_SUBDIRECTORY(ACADOtoolkit)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()


find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package(
	#INCLUDE_DIRS_acado ACADOtoolkit/acado
	#INCLUDE_DIRS_toolkit ACADOtoolkit

	INCLUDE_DIRS ACADOtoolkit
)







