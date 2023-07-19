# Install script for directory: /home/saob/MySource/uav_racer/src/airsim_ros_pkgs

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
    set(CMAKE_INSTALL_CONFIG_NAME "Debug")
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

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/airsim_ros_pkgs/msg" TYPE FILE FILES
    "/home/saob/MySource/uav_racer/src/airsim_ros_pkgs/msg/GimbalAngleEulerCmd.msg"
    "/home/saob/MySource/uav_racer/src/airsim_ros_pkgs/msg/GimbalAngleQuatCmd.msg"
    "/home/saob/MySource/uav_racer/src/airsim_ros_pkgs/msg/GPSYaw.msg"
    "/home/saob/MySource/uav_racer/src/airsim_ros_pkgs/msg/VelCmd.msg"
    "/home/saob/MySource/uav_racer/src/airsim_ros_pkgs/msg/VelCmdGroup.msg"
    "/home/saob/MySource/uav_racer/src/airsim_ros_pkgs/msg/CarControls.msg"
    "/home/saob/MySource/uav_racer/src/airsim_ros_pkgs/msg/CarState.msg"
    "/home/saob/MySource/uav_racer/src/airsim_ros_pkgs/msg/Altimeter.msg"
    "/home/saob/MySource/uav_racer/src/airsim_ros_pkgs/msg/Environment.msg"
    "/home/saob/MySource/uav_racer/src/airsim_ros_pkgs/msg/PoseCmd.msg"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/airsim_ros_pkgs/srv" TYPE FILE FILES
    "/home/saob/MySource/uav_racer/src/airsim_ros_pkgs/srv/SetGPSPosition.srv"
    "/home/saob/MySource/uav_racer/src/airsim_ros_pkgs/srv/Takeoff.srv"
    "/home/saob/MySource/uav_racer/src/airsim_ros_pkgs/srv/TakeoffGroup.srv"
    "/home/saob/MySource/uav_racer/src/airsim_ros_pkgs/srv/Land.srv"
    "/home/saob/MySource/uav_racer/src/airsim_ros_pkgs/srv/LandGroup.srv"
    "/home/saob/MySource/uav_racer/src/airsim_ros_pkgs/srv/Reset.srv"
    "/home/saob/MySource/uav_racer/src/airsim_ros_pkgs/srv/SetLocalPosition.srv"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/airsim_ros_pkgs/cmake" TYPE FILE FILES "/home/saob/MySource/uav_racer/src/cmake-build-debug/airsim_ros_pkgs/catkin_generated/installspace/airsim_ros_pkgs-msg-paths.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/saob/MySource/uav_racer/src/cmake-build-debug/devel/include/airsim_ros_pkgs")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/saob/MySource/uav_racer/src/cmake-build-debug/devel/share/roseus/ros/airsim_ros_pkgs")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/saob/MySource/uav_racer/src/cmake-build-debug/devel/share/common-lisp/ros/airsim_ros_pkgs")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/saob/MySource/uav_racer/src/cmake-build-debug/devel/share/gennodejs/ros/airsim_ros_pkgs")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/home/saob/MySource/uav_racer/src/cmake-build-debug/devel/lib/python3/dist-packages/airsim_ros_pkgs")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/saob/MySource/uav_racer/src/cmake-build-debug/devel/lib/python3/dist-packages/airsim_ros_pkgs")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/saob/MySource/uav_racer/src/cmake-build-debug/airsim_ros_pkgs/catkin_generated/installspace/airsim_ros_pkgs.pc")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/airsim_ros_pkgs/cmake" TYPE FILE FILES "/home/saob/MySource/uav_racer/src/cmake-build-debug/airsim_ros_pkgs/catkin_generated/installspace/airsim_ros_pkgs-msg-extras.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/airsim_ros_pkgs/cmake" TYPE FILE FILES
    "/home/saob/MySource/uav_racer/src/cmake-build-debug/airsim_ros_pkgs/catkin_generated/installspace/airsim_ros_pkgsConfig.cmake"
    "/home/saob/MySource/uav_racer/src/cmake-build-debug/airsim_ros_pkgs/catkin_generated/installspace/airsim_ros_pkgsConfig-version.cmake"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/airsim_ros_pkgs" TYPE FILE FILES "/home/saob/MySource/uav_racer/src/airsim_ros_pkgs/package.xml")
endif()

