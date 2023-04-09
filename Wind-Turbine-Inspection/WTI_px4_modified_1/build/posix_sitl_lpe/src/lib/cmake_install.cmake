# Install script for directory: /home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/src/lib

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "RelWithDebInfo")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/build/posix_sitl_lpe/src/lib/airspeed/cmake_install.cmake")
  include("/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/build/posix_sitl_lpe/src/lib/battery/cmake_install.cmake")
  include("/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/build/posix_sitl_lpe/src/lib/circuit_breaker/cmake_install.cmake")
  include("/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/build/posix_sitl_lpe/src/lib/controllib/cmake_install.cmake")
  include("/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/build/posix_sitl_lpe/src/lib/conversion/cmake_install.cmake")
  include("/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/build/posix_sitl_lpe/src/lib/drivers/cmake_install.cmake")
  include("/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/build/posix_sitl_lpe/src/lib/ecl/cmake_install.cmake")
  include("/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/build/posix_sitl_lpe/src/lib/FlightTasks/cmake_install.cmake")
  include("/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/build/posix_sitl_lpe/src/lib/led/cmake_install.cmake")
  include("/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/build/posix_sitl_lpe/src/lib/mathlib/cmake_install.cmake")
  include("/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/build/posix_sitl_lpe/src/lib/mixer/cmake_install.cmake")
  include("/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/build/posix_sitl_lpe/src/lib/perf/cmake_install.cmake")
  include("/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/build/posix_sitl_lpe/src/lib/pid/cmake_install.cmake")
  include("/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/build/posix_sitl_lpe/src/lib/pwm_limit/cmake_install.cmake")
  include("/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/build/posix_sitl_lpe/src/lib/rc/cmake_install.cmake")
  include("/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/build/posix_sitl_lpe/src/lib/terrain_estimation/cmake_install.cmake")
  include("/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/build/posix_sitl_lpe/src/lib/tunes/cmake_install.cmake")
  include("/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/build/posix_sitl_lpe/src/lib/version/cmake_install.cmake")

endif()

