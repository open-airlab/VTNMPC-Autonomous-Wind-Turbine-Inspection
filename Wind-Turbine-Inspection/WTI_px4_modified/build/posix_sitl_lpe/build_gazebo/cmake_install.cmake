# Install script for directory: /home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/Tools/sitl_gazebo

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
    set(CMAKE_INSTALL_CONFIG_NAME "")
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

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
        file(MAKE_DIRECTORY "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
      endif()
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin")
        file(WRITE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin" "")
      endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/_setup_util.py")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local" TYPE PROGRAM FILES "/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/build/posix_sitl_lpe/build_gazebo/catkin_generated/installspace/_setup_util.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/env.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local" TYPE PROGRAM FILES "/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/build/posix_sitl_lpe/build_gazebo/catkin_generated/installspace/env.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/setup.bash;/usr/local/local_setup.bash")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local" TYPE FILE FILES
    "/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/build/posix_sitl_lpe/build_gazebo/catkin_generated/installspace/setup.bash"
    "/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/build/posix_sitl_lpe/build_gazebo/catkin_generated/installspace/local_setup.bash"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/setup.sh;/usr/local/local_setup.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local" TYPE FILE FILES
    "/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/build/posix_sitl_lpe/build_gazebo/catkin_generated/installspace/setup.sh"
    "/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/build/posix_sitl_lpe/build_gazebo/catkin_generated/installspace/local_setup.sh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/setup.zsh;/usr/local/local_setup.zsh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local" TYPE FILE FILES
    "/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/build/posix_sitl_lpe/build_gazebo/catkin_generated/installspace/setup.zsh"
    "/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/build/posix_sitl_lpe/build_gazebo/catkin_generated/installspace/local_setup.zsh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/.rosinstall")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local" TYPE FILE FILES "/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/build/posix_sitl_lpe/build_gazebo/catkin_generated/installspace/.rosinstall")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/build/posix_sitl_lpe/build_gazebo/catkin_generated/installspace/mavlink_sitl_gazebo.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mavlink_sitl_gazebo/cmake" TYPE FILE FILES
    "/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/build/posix_sitl_lpe/build_gazebo/catkin_generated/installspace/mavlink_sitl_gazeboConfig.cmake"
    "/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/build/posix_sitl_lpe/build_gazebo/catkin_generated/installspace/mavlink_sitl_gazeboConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mavlink_sitl_gazebo" TYPE FILE FILES "/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/Tools/sitl_gazebo/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/etc/catkin/profile.d" TYPE FILE FILES "/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/build/posix_sitl_lpe/build_gazebo/catkin_generated/installspace/50_sitl_gazebo_setup.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mavlink_sitl_gazebo/catkin_env_hook" TYPE FILE FILES "/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/build/posix_sitl_lpe/build_gazebo/catkin_generated/installspace/50_sitl_gazebo_setup.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libgazebo_geotagged_images_plugin.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libgazebo_geotagged_images_plugin.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libgazebo_geotagged_images_plugin.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins" TYPE SHARED_LIBRARY FILES "/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/build/posix_sitl_lpe/build_gazebo/devel/lib/libgazebo_geotagged_images_plugin.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libgazebo_geotagged_images_plugin.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libgazebo_geotagged_images_plugin.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libgazebo_geotagged_images_plugin.so"
         OLD_RPATH "/usr/lib/x86_64-linux-gnu/gazebo-9/plugins:/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/build/posix_sitl_lpe/build_gazebo:/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/build/posix_sitl_lpe/build_gazebo/devel/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libgazebo_geotagged_images_plugin.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libgazebo_gps_plugin.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libgazebo_gps_plugin.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libgazebo_gps_plugin.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins" TYPE SHARED_LIBRARY FILES "/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/build/posix_sitl_lpe/build_gazebo/devel/lib/libgazebo_gps_plugin.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libgazebo_gps_plugin.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libgazebo_gps_plugin.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libgazebo_gps_plugin.so"
         OLD_RPATH "/usr/lib/x86_64-linux-gnu/gazebo-9/plugins:/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/build/posix_sitl_lpe/build_gazebo:/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/build/posix_sitl_lpe/build_gazebo/devel/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libgazebo_gps_plugin.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libgazebo_irlock_plugin.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libgazebo_irlock_plugin.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libgazebo_irlock_plugin.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins" TYPE SHARED_LIBRARY FILES "/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/build/posix_sitl_lpe/build_gazebo/devel/lib/libgazebo_irlock_plugin.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libgazebo_irlock_plugin.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libgazebo_irlock_plugin.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libgazebo_irlock_plugin.so"
         OLD_RPATH "/usr/lib/x86_64-linux-gnu/gazebo-9/plugins:/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/build/posix_sitl_lpe/build_gazebo:/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/build/posix_sitl_lpe/build_gazebo/devel/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libgazebo_irlock_plugin.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libgazebo_lidar_plugin.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libgazebo_lidar_plugin.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libgazebo_lidar_plugin.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins" TYPE SHARED_LIBRARY FILES "/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/build/posix_sitl_lpe/build_gazebo/devel/lib/libgazebo_lidar_plugin.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libgazebo_lidar_plugin.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libgazebo_lidar_plugin.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libgazebo_lidar_plugin.so"
         OLD_RPATH "/usr/lib/x86_64-linux-gnu/gazebo-9/plugins:/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/build/posix_sitl_lpe/build_gazebo:/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/build/posix_sitl_lpe/build_gazebo/devel/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libgazebo_lidar_plugin.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libgazebo_opticalflow_plugin.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libgazebo_opticalflow_plugin.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libgazebo_opticalflow_plugin.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins" TYPE SHARED_LIBRARY FILES "/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/build/posix_sitl_lpe/build_gazebo/devel/lib/libgazebo_opticalflow_plugin.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libgazebo_opticalflow_plugin.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libgazebo_opticalflow_plugin.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libgazebo_opticalflow_plugin.so"
         OLD_RPATH "/usr/lib/x86_64-linux-gnu/gazebo-9/plugins:/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/build/posix_sitl_lpe/build_gazebo:/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/build/posix_sitl_lpe/build_gazebo/devel/lib:/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/build/posix_sitl_lpe/build_gazebo/OpticalFlow:/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/build/posix_sitl_lpe/build_gazebo/OpticalFlow/klt_feature_tracker:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libgazebo_opticalflow_plugin.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libgazebo_sonar_plugin.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libgazebo_sonar_plugin.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libgazebo_sonar_plugin.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins" TYPE SHARED_LIBRARY FILES "/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/build/posix_sitl_lpe/build_gazebo/devel/lib/libgazebo_sonar_plugin.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libgazebo_sonar_plugin.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libgazebo_sonar_plugin.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libgazebo_sonar_plugin.so"
         OLD_RPATH "/usr/lib/x86_64-linux-gnu/gazebo-9/plugins:/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/build/posix_sitl_lpe/build_gazebo:/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/build/posix_sitl_lpe/build_gazebo/devel/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libgazebo_sonar_plugin.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libgazebo_uuv_plugin.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libgazebo_uuv_plugin.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libgazebo_uuv_plugin.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins" TYPE SHARED_LIBRARY FILES "/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/build/posix_sitl_lpe/build_gazebo/devel/lib/libgazebo_uuv_plugin.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libgazebo_uuv_plugin.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libgazebo_uuv_plugin.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libgazebo_uuv_plugin.so"
         OLD_RPATH "/usr/lib/x86_64-linux-gnu/gazebo-9/plugins:/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/build/posix_sitl_lpe/build_gazebo:/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/build/posix_sitl_lpe/build_gazebo/devel/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libgazebo_uuv_plugin.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libgazebo_vision_plugin.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libgazebo_vision_plugin.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libgazebo_vision_plugin.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins" TYPE SHARED_LIBRARY FILES "/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/build/posix_sitl_lpe/build_gazebo/devel/lib/libgazebo_vision_plugin.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libgazebo_vision_plugin.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libgazebo_vision_plugin.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libgazebo_vision_plugin.so"
         OLD_RPATH "/usr/lib/x86_64-linux-gnu/gazebo-9/plugins:/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/build/posix_sitl_lpe/build_gazebo:/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/build/posix_sitl_lpe/build_gazebo/devel/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libgazebo_vision_plugin.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libgazebo_controller_interface.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libgazebo_controller_interface.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libgazebo_controller_interface.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins" TYPE SHARED_LIBRARY FILES "/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/build/posix_sitl_lpe/build_gazebo/devel/lib/libgazebo_controller_interface.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libgazebo_controller_interface.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libgazebo_controller_interface.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libgazebo_controller_interface.so"
         OLD_RPATH "/usr/lib/x86_64-linux-gnu/gazebo-9/plugins:/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/build/posix_sitl_lpe/build_gazebo:/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/build/posix_sitl_lpe/build_gazebo/devel/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libgazebo_controller_interface.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libgazebo_gimbal_controller_plugin.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libgazebo_gimbal_controller_plugin.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libgazebo_gimbal_controller_plugin.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins" TYPE SHARED_LIBRARY FILES "/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/build/posix_sitl_lpe/build_gazebo/devel/lib/libgazebo_gimbal_controller_plugin.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libgazebo_gimbal_controller_plugin.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libgazebo_gimbal_controller_plugin.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libgazebo_gimbal_controller_plugin.so"
         OLD_RPATH "/usr/lib/x86_64-linux-gnu/gazebo-9/plugins:/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/build/posix_sitl_lpe/build_gazebo:/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/build/posix_sitl_lpe/build_gazebo/devel/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libgazebo_gimbal_controller_plugin.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libgazebo_imu_plugin.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libgazebo_imu_plugin.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libgazebo_imu_plugin.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins" TYPE SHARED_LIBRARY FILES "/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/build/posix_sitl_lpe/build_gazebo/devel/lib/libgazebo_imu_plugin.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libgazebo_imu_plugin.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libgazebo_imu_plugin.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libgazebo_imu_plugin.so"
         OLD_RPATH "/usr/lib/x86_64-linux-gnu/gazebo-9/plugins:/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/build/posix_sitl_lpe/build_gazebo:/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/build/posix_sitl_lpe/build_gazebo/devel/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libgazebo_imu_plugin.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libgazebo_mavlink_interface.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libgazebo_mavlink_interface.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libgazebo_mavlink_interface.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins" TYPE SHARED_LIBRARY FILES "/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/build/posix_sitl_lpe/build_gazebo/devel/lib/libgazebo_mavlink_interface.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libgazebo_mavlink_interface.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libgazebo_mavlink_interface.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libgazebo_mavlink_interface.so"
         OLD_RPATH "/usr/lib/x86_64-linux-gnu/gazebo-9/plugins:/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/build/posix_sitl_lpe/build_gazebo:/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/build/posix_sitl_lpe/build_gazebo/devel/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libgazebo_mavlink_interface.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libgazebo_motor_model.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libgazebo_motor_model.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libgazebo_motor_model.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins" TYPE SHARED_LIBRARY FILES "/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/build/posix_sitl_lpe/build_gazebo/devel/lib/libgazebo_motor_model.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libgazebo_motor_model.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libgazebo_motor_model.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libgazebo_motor_model.so"
         OLD_RPATH "/usr/lib/x86_64-linux-gnu/gazebo-9/plugins:/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/build/posix_sitl_lpe/build_gazebo:/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/build/posix_sitl_lpe/build_gazebo/devel/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libgazebo_motor_model.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libgazebo_multirotor_base_plugin.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libgazebo_multirotor_base_plugin.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libgazebo_multirotor_base_plugin.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins" TYPE SHARED_LIBRARY FILES "/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/build/posix_sitl_lpe/build_gazebo/devel/lib/libgazebo_multirotor_base_plugin.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libgazebo_multirotor_base_plugin.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libgazebo_multirotor_base_plugin.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libgazebo_multirotor_base_plugin.so"
         OLD_RPATH "/usr/lib/x86_64-linux-gnu/gazebo-9/plugins:/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/build/posix_sitl_lpe/build_gazebo:/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/build/posix_sitl_lpe/build_gazebo/devel/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libgazebo_multirotor_base_plugin.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libgazebo_wind_plugin.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libgazebo_wind_plugin.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libgazebo_wind_plugin.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins" TYPE SHARED_LIBRARY FILES "/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/build/posix_sitl_lpe/build_gazebo/devel/lib/libgazebo_wind_plugin.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libgazebo_wind_plugin.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libgazebo_wind_plugin.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libgazebo_wind_plugin.so"
         OLD_RPATH "/usr/lib/x86_64-linux-gnu/gazebo-9/plugins:/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/build/posix_sitl_lpe/build_gazebo:/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/build/posix_sitl_lpe/build_gazebo/devel/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libgazebo_wind_plugin.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libgazebo_motor_failure_plugin.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libgazebo_motor_failure_plugin.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libgazebo_motor_failure_plugin.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins" TYPE SHARED_LIBRARY FILES "/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/build/posix_sitl_lpe/build_gazebo/devel/lib/libgazebo_motor_failure_plugin.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libgazebo_motor_failure_plugin.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libgazebo_motor_failure_plugin.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libgazebo_motor_failure_plugin.so"
         OLD_RPATH "/usr/lib/x86_64-linux-gnu/gazebo-9/plugins:/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/build/posix_sitl_lpe/build_gazebo:/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/build/posix_sitl_lpe/build_gazebo/devel/lib:/opt/ros/melodic/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libgazebo_motor_failure_plugin.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/librosgazebo_wind_plugin.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/librosgazebo_wind_plugin.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/librosgazebo_wind_plugin.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins" TYPE SHARED_LIBRARY FILES "/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/build/posix_sitl_lpe/build_gazebo/devel/lib/librosgazebo_wind_plugin.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/librosgazebo_wind_plugin.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/librosgazebo_wind_plugin.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/librosgazebo_wind_plugin.so"
         OLD_RPATH "/usr/lib/x86_64-linux-gnu/gazebo-9/plugins:/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/build/posix_sitl_lpe/build_gazebo:/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/build/posix_sitl_lpe/build_gazebo/devel/lib:/opt/ros/melodic/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/librosgazebo_wind_plugin.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/librosgazebo_lidar_plugin.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/librosgazebo_lidar_plugin.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/librosgazebo_lidar_plugin.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins" TYPE SHARED_LIBRARY FILES "/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/build/posix_sitl_lpe/build_gazebo/devel/lib/librosgazebo_lidar_plugin.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/librosgazebo_lidar_plugin.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/librosgazebo_lidar_plugin.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/librosgazebo_lidar_plugin.so"
         OLD_RPATH "/usr/lib/x86_64-linux-gnu/gazebo-9/plugins:/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/build/posix_sitl_lpe/build_gazebo:/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/build/posix_sitl_lpe/build_gazebo/devel/lib:/opt/ros/melodic/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/librosgazebo_lidar_plugin.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libLiftDragPlugin.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libLiftDragPlugin.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libLiftDragPlugin.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins" TYPE SHARED_LIBRARY FILES "/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/build/posix_sitl_lpe/build_gazebo/devel/lib/libLiftDragPlugin.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libLiftDragPlugin.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libLiftDragPlugin.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libLiftDragPlugin.so"
         OLD_RPATH "/usr/lib/x86_64-linux-gnu/gazebo-9/plugins:/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/build/posix_sitl_lpe/build_gazebo:/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/build/posix_sitl_lpe/build_gazebo/devel/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libLiftDragPlugin.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libmav_msgs.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libmav_msgs.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libmav_msgs.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins" TYPE SHARED_LIBRARY FILES "/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/build/posix_sitl_lpe/build_gazebo/devel/lib/libmav_msgs.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libmav_msgs.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libmav_msgs.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libmav_msgs.so"
         OLD_RPATH "/usr/lib/x86_64-linux-gnu/gazebo-9/plugins:/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/build/posix_sitl_lpe/build_gazebo:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libmav_msgs.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libnav_msgs.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libnav_msgs.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libnav_msgs.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins" TYPE SHARED_LIBRARY FILES "/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/build/posix_sitl_lpe/build_gazebo/devel/lib/libnav_msgs.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libnav_msgs.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libnav_msgs.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libnav_msgs.so"
         OLD_RPATH "/usr/lib/x86_64-linux-gnu/gazebo-9/plugins:/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/build/posix_sitl_lpe/build_gazebo:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libnav_msgs.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libstd_msgs.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libstd_msgs.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libstd_msgs.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins" TYPE SHARED_LIBRARY FILES "/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/build/posix_sitl_lpe/build_gazebo/devel/lib/libstd_msgs.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libstd_msgs.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libstd_msgs.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libstd_msgs.so"
         OLD_RPATH "/usr/lib/x86_64-linux-gnu/gazebo-9/plugins:/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/build/posix_sitl_lpe/build_gazebo:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libstd_msgs.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libsensor_msgs.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libsensor_msgs.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libsensor_msgs.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins" TYPE SHARED_LIBRARY FILES "/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/build/posix_sitl_lpe/build_gazebo/devel/lib/libsensor_msgs.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libsensor_msgs.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libsensor_msgs.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libsensor_msgs.so"
         OLD_RPATH "/usr/lib/x86_64-linux-gnu/gazebo-9/plugins:/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/build/posix_sitl_lpe/build_gazebo:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mavlink_sitl_gazebo/plugins/libsensor_msgs.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mavlink_sitl_gazebo/models" TYPE DIRECTORY FILES
    "/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/Tools/sitl_gazebo/models/3DR_gps_mag"
    "/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/Tools/sitl_gazebo/models/Box"
    "/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/Tools/sitl_gazebo/models/BoxesLargeOnPallet"
    "/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/Tools/sitl_gazebo/models/BoxesLargeOnPallet_2"
    "/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/Tools/sitl_gazebo/models/BoxesLargeOnPallet_3"
    "/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/Tools/sitl_gazebo/models/asphalt_plane"
    "/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/Tools/sitl_gazebo/models/big_box"
    "/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/Tools/sitl_gazebo/models/big_box2"
    "/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/Tools/sitl_gazebo/models/big_box3"
    "/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/Tools/sitl_gazebo/models/big_box4"
    "/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/Tools/sitl_gazebo/models/c920"
    "/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/Tools/sitl_gazebo/models/chalk_wall"
    "/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/Tools/sitl_gazebo/models/delta_wing"
    "/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/Tools/sitl_gazebo/models/ellipsoid_nmpc"
    "/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/Tools/sitl_gazebo/models/ellipsoid_obst_1"
    "/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/Tools/sitl_gazebo/models/ellipsoid_obst_2"
    "/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/Tools/sitl_gazebo/models/europallet"
    "/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/Tools/sitl_gazebo/models/flow_cam"
    "/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/Tools/sitl_gazebo/models/fpv_cam"
    "/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/Tools/sitl_gazebo/models/ground_plane"
    "/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/Tools/sitl_gazebo/models/hippocampus"
    "/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/Tools/sitl_gazebo/models/iris"
    "/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/Tools/sitl_gazebo/models/iris_fpv_cam"
    "/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/Tools/sitl_gazebo/models/iris_irlock"
    "/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/Tools/sitl_gazebo/models/iris_opt_flow"
    "/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/Tools/sitl_gazebo/models/iris_rplidar"
    "/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/Tools/sitl_gazebo/models/iris_vision"
    "/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/Tools/sitl_gazebo/models/irlock"
    "/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/Tools/sitl_gazebo/models/lidar"
    "/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/Tools/sitl_gazebo/models/matrice_100"
    "/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/Tools/sitl_gazebo/models/matrice_100_opt_flow"
    "/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/Tools/sitl_gazebo/models/mb1240-xl-ez4"
    "/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/Tools/sitl_gazebo/models/pallet_full"
    "/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/Tools/sitl_gazebo/models/pixhawk"
    "/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/Tools/sitl_gazebo/models/plane"
    "/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/Tools/sitl_gazebo/models/polaris_ranger_ev"
    "/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/Tools/sitl_gazebo/models/px4flow"
    "/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/Tools/sitl_gazebo/models/r200"
    "/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/Tools/sitl_gazebo/models/ramp"
    "/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/Tools/sitl_gazebo/models/rotors_description"
    "/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/Tools/sitl_gazebo/models/rover"
    "/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/Tools/sitl_gazebo/models/rplidar"
    "/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/Tools/sitl_gazebo/models/sf10a"
    "/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/Tools/sitl_gazebo/models/shelves_high"
    "/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/Tools/sitl_gazebo/models/shelves_high2"
    "/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/Tools/sitl_gazebo/models/shelves_high2_no_collision"
    "/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/Tools/sitl_gazebo/models/small_box"
    "/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/Tools/sitl_gazebo/models/solo"
    "/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/Tools/sitl_gazebo/models/sonar"
    "/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/Tools/sitl_gazebo/models/standard_vtol"
    "/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/Tools/sitl_gazebo/models/sun"
    "/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/Tools/sitl_gazebo/models/tailsitter"
    "/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/Tools/sitl_gazebo/models/tiltrotor"
    "/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/Tools/sitl_gazebo/models/typhoon_h480"
    "/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/Tools/sitl_gazebo/models/uneven_ground"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mavlink_sitl_gazebo/worlds" TYPE FILE FILES
    "/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/Tools/sitl_gazebo/worlds/delta_wing.world"
    "/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/Tools/sitl_gazebo/worlds/empty.world"
    "/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/Tools/sitl_gazebo/worlds/hippocampus.world"
    "/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/Tools/sitl_gazebo/worlds/iris.world"
    "/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/Tools/sitl_gazebo/worlds/iris_fpv_cam.world"
    "/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/Tools/sitl_gazebo/worlds/iris_irlock.world"
    "/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/Tools/sitl_gazebo/worlds/iris_opt_flow.world"
    "/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/Tools/sitl_gazebo/worlds/iris_rplidar.world"
    "/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/Tools/sitl_gazebo/worlds/iris_vision.world"
    "/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/Tools/sitl_gazebo/worlds/matrice_100.world"
    "/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/Tools/sitl_gazebo/worlds/matrice_100_opt_flow.world"
    "/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/Tools/sitl_gazebo/worlds/matrice_100_wall.world"
    "/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/Tools/sitl_gazebo/worlds/plane.world"
    "/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/Tools/sitl_gazebo/worlds/rover.world"
    "/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/Tools/sitl_gazebo/worlds/rubble.world"
    "/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/Tools/sitl_gazebo/worlds/solo.world"
    "/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/Tools/sitl_gazebo/worlds/standard_vtol.world"
    "/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/Tools/sitl_gazebo/worlds/tailsitter.world"
    "/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/Tools/sitl_gazebo/worlds/tiltrotor.world"
    "/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/Tools/sitl_gazebo/worlds/typhoon_h480.world"
    "/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/Tools/sitl_gazebo/worlds/uneven.world"
    "/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/Tools/sitl_gazebo/worlds/warehouse.world"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mavlink_sitl_gazebo" TYPE FILE FILES "/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/build/posix_sitl_lpe/build_gazebo/setup.sh")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/build/posix_sitl_lpe/build_gazebo/OpticalFlow/cmake_install.cmake")
  include("/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/build/posix_sitl_lpe/build_gazebo/gtest/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/hakim/Wind-Turbine-Inspection/WTI_px4_modified/build/posix_sitl_lpe/build_gazebo/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
