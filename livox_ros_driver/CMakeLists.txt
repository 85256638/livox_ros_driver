# ====================================================================
# livox_ros_driver  —  CMakeLists.txt   (for Livox-SDK static library)
# Tested on: Ubuntu 20.04 + ROS Noetic + Livox-SDK (Horizon)
# ====================================================================

cmake_minimum_required(VERSION 3.0)
include(cmake/version.cmake)
project(livox_ros_driver VERSION ${LIVOX_ROS_DRIVER_VERSION} LANGUAGES CXX)
message(STATUS "livox_ros_driver version: ${LIVOX_ROS_DRIVER_VERSION}")

# --------------------------------------------------------------------
# 1. Dependencies
# --------------------------------------------------------------------
find_package(Boost 1.54 REQUIRED COMPONENTS system thread chrono)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  message_generation
  rosbag
  pcl_ros
  livox_mode_switcher
)

find_package(PCL REQUIRED)
find_package(PkgConfig)
pkg_check_modules(APR apr-1)

# --------------------------------------------------------------------
# 2. Message generation
# --------------------------------------------------------------------
add_message_files(
  FILES
    CustomPoint.msg
    CustomMsg.msg
)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    rospy
    std_msgs
    message_runtime
    pcl_ros
    livox_mode_switcher
)

# --------------------------------------------------------------------
# 3. Build type & compiler
# --------------------------------------------------------------------
if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Debug or Release" FORCE)
endif()
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --------------------------------------------------------------------
# 4. Livox-SDK include & library (STATIC)
# --------------------------------------------------------------------
set(LIVOX_SDK_INC  "/home/xlkj/Livox-SDK/sdk_core/include")
set(LIVOX_SDK_LIB  "/home/xlkj/Livox-SDK/build/sdk_core/liblivox_sdk_static.a")

# Sanity check
if (NOT EXISTS ${LIVOX_SDK_INC}/livox_sdk.h OR NOT EXISTS ${LIVOX_SDK_LIB})
  message(FATAL_ERROR "Livox-SDK headers or library not found at:\n  ${LIVOX_SDK_INC}\n  ${LIVOX_SDK_LIB}")
endif()

include_directories(BEFORE ${LIVOX_SDK_INC})
link_directories(/home/xlkj/Livox-SDK/build/sdk_core)

# --------------------------------------------------------------------
# 5. Source files
# --------------------------------------------------------------------
set(LIVOX_DRIVER_SRCS
  livox_ros_driver/lvx_file.cpp
  livox_ros_driver/ldq.cpp
  livox_ros_driver/lds.cpp
  livox_ros_driver/lds_lvx.cpp
  livox_ros_driver/lds_lidar.cpp
  livox_ros_driver/lds_hub.cpp
  livox_ros_driver/lddc.cpp
  livox_ros_driver/livox_ros_driver.cpp
  timesync/timesync.cpp
  timesync/user_uart/user_uart.cpp
  common/comm/comm_protocol.cpp
  common/comm/sdk_protocol.cpp
  common/comm/gps_protocol.cpp
)

# --------------------------------------------------------------------
# 6. Executable target
# --------------------------------------------------------------------
add_executable(${PROJECT_NAME}_node ${LIVOX_DRIVER_SRCS})

target_include_directories(${PROJECT_NAME}_node PUBLIC
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${APR_INCLUDE_DIRS}
  common
  common/rapidjson
  common/comm
  timesync
  timesync/user_uart
  livox_ros_driver
)

target_link_libraries(${PROJECT_NAME}_node
  ${LIVOX_SDK_LIB}          # <-- Livox-SDK 静态库
  pthread
  usb-1.0
  rt                        # <-- 静态库所需系统依赖
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${APR_LIBRARIES}
)

add_dependencies(${PROJECT_NAME}_node
  ${PROJECT_NAME}_generate_messages_cpp
)

# --------------------------------------------------------------------
# 7. Install
# --------------------------------------------------------------------
install(TARGETS ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

