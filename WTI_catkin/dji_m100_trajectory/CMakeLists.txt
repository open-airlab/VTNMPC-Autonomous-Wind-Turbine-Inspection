cmake_minimum_required(VERSION 2.8.3)
project(dji_m100_trajectory)
set(PROJECT_NAME dji_m100_trajectory)
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

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs roslib nav_msgs mavros_msgs tf dynamic_reconfigure
)

generate_dynamic_reconfigure_options(cfg/set_trajectory.cfg cfg/set_trajectory_v2.cfg cfg/set_dummyLidar.cfg cfg/set_wind_generation.cfg cfg/set_recdata.cfg)

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

if(true)

include_directories(
  ${catkin_INCLUDE_DIRS}

#  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/include
)


############
# BINARIES #
############

add_executable(m100_trajectory src/trajectory.cpp)
target_link_libraries(m100_trajectory ${catkin_LIBRARIES})
add_dependencies(m100_trajectory ${PROJECT_NAME}_gencfg)

add_executable(m100_trajectory_v2 src/trajectory_v2.cpp)
target_link_libraries(m100_trajectory_v2 ${catkin_LIBRARIES})
add_dependencies(m100_trajectory_v2 ${PROJECT_NAME}_gencfg)

add_executable(dummyLidar src/dummyLidar.cpp)
target_link_libraries(dummyLidar ${catkin_LIBRARIES})
add_dependencies(dummyLidar ${PROJECT_NAME}_gencfg)

add_executable(wind_generation src/wind_generation.cpp)
target_link_libraries(wind_generation ${catkin_LIBRARIES})
add_dependencies(wind_generation ${PROJECT_NAME}_gencfg)

add_executable(GP_statemachine src/GP_statemachine.cpp)
target_link_libraries(GP_statemachine ${catkin_LIBRARIES})
add_dependencies(GP_statemachine ${PROJECT_NAME}_gencfg)


add_executable(point_node src/point_node.cpp)
target_link_libraries(point_node ${catkin_LIBRARIES})
add_dependencies(point_node ${PROJECT_NAME}_gencfg)

add_executable(perception_test_node src/perception_test.cpp)
target_link_libraries(perception_test_node ${catkin_LIBRARIES})
add_dependencies(perception_test_node ${PROJECT_NAME}_gencfg)







add_executable(recdata src/recdata.cpp)
target_link_libraries(recdata ${catkin_LIBRARIES})
add_dependencies(recdata ${PROJECT_NAME}_gencfg)

endif()
