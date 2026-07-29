# Copyright(c) 2019 livoxtech limited.

cmake_minimum_required(VERSION 3.0)


#---------------------------------------------------------------------------------------
# Start livox_ros_driver project
#---------------------------------------------------------------------------------------
include(cmake/version.cmake)
project(livox_ros_driver VERSION ${LIVOX_ROS_DRIVER_VERSION} LANGUAGES CXX)
message(STATUS "livox_ros_driver version: ${LIVOX_ROS_DRIVER_VERSION}")

#---------------------------------------------------------------------------------------
# find package and the dependecy
#---------------------------------------------------------------------------------------
find_package(Boost 1.54 REQUIRED COMPONENTS
	system
	thread
	chrono
	)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	sensor_msgs
	std_msgs
	message_generation
	rosbag
	pcl_ros
	)

## Find pcl lib
find_package(PCL REQUIRED)

## Generate messages in the 'msg' folder
add_message_files(FILES
	CustomPoint.msg
	CustomMsg.msg
	)

## Generate services in the 'srv' folder
add_service_files(FILES
	LidarMode.srv
	LidarReboot.srv
	)

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES
	std_msgs
	)

find_package(PkgConfig)
pkg_check_modules(APR apr-1)
if (APR_FOUND)
	message(${APR_INCLUDE_DIRS})
	message(${APR_LIBRARIES})
endif (APR_FOUND)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects als    o need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also n    eed
catkin_package(CATKIN_DEPENDS
	roscpp rospy std_msgs message_runtime
	pcl_ros
	)

## Install python scripts so they are runnable via `rosrun livox_ros_driver <name>`
catkin_install_python(PROGRAMS
	livox_ros_driver/scripts/livox_stats_monitor.py
	livox_ros_driver/scripts/livox_power_cycle_manager.py
	livox_ros_driver/scripts/validate_livox_power_cycle_site.py
	livox_ros_driver/scripts/merge_livox_relay_launch.py
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#---------------------------------------------------------------------------------------
# Set default build to release
#---------------------------------------------------------------------------------------
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose Release or Debug" FORCE)
endif()

#---------------------------------------------------------------------------------------
# Compiler config
#---------------------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(cmake/pinned_livox_sdk.cmake)

## PCL library
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#---------------------------------------------------------------------------------------
# generate excutable and add libraries
#---------------------------------------------------------------------------------------
add_executable(${PROJECT_NAME}_node
    ""
    )

#---------------------------------------------------------------------------------------
# precompile macro and compile option
#---------------------------------------------------------------------------------------
target_compile_options(${PROJECT_NAME}_node
    PRIVATE $<$<CXX_COMPILER_ID:GNU>:-Wall>
    )

#---------------------------------------------------------------------------------------
# add projects that depend on
#---------------------------------------------------------------------------------------
add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_generate_messages_cpp)

#---------------------------------------------------------------------------------------
# source file
#---------------------------------------------------------------------------------------
target_sources(${PROJECT_NAME}_node
	PRIVATE
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

#---------------------------------------------------------------------------------------
# include file
#---------------------------------------------------------------------------------------
target_include_directories(${PROJECT_NAME}_node
    PUBLIC
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${APR_INCLUDE_DIRS}
    common
    common/rapidjson
    comon/rapdidxml
    common/comm
    timesync
    timesync/user_uart
    livox_ros_driver
    )

#---------------------------------------------------------------------------------------
# link libraries
#---------------------------------------------------------------------------------------
target_link_libraries(${PROJECT_NAME}_node
	livox_sdk_static
	${Boost_LIBRARY}
	${catkin_LIBRARIES}
	${PCL_LIBRARIES}
	${APR_LIBRARIES}
	)

#---------------------------------------------------------------------------------------
# Recovery safety quality gates
#---------------------------------------------------------------------------------------
if(CATKIN_ENABLE_TESTING)
  set(LIVOX_RECOVERY_QC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../tests")
  set(LIVOX_RECOVERY_QC_TARGETS
    dashboard_metrics_qc
    normal_dropout_policy_qc
    wake_dropout_policy_qc
    recovery_event_json_qc
    group_power_cycle_protocol_qc
  )
  foreach(qc_target IN LISTS LIVOX_RECOVERY_QC_TARGETS)
    add_executable(${qc_target} "${LIVOX_RECOVERY_QC_DIR}/${qc_target}.cpp")
    target_include_directories(${qc_target} PRIVATE
      "${CMAKE_CURRENT_SOURCE_DIR}/livox_ros_driver"
      "${CMAKE_CURRENT_SOURCE_DIR}/common"
    )
    add_test(NAME ${qc_target} COMMAND $<TARGET_FILE:${qc_target}>)
    set_tests_properties(${qc_target} PROPERTIES LABELS "livox_recovery_qc")
  endforeach()

  if(NOT PYTHON_EXECUTABLE)
    find_package(PythonInterp REQUIRED)
  endif()
  set(LIVOX_RECOVERY_PY_TESTS
    test_handshake_recovery_policy
    test_livox_power_cycle_manager
    test_livox_stats_monitor
    test_merge_livox_relay_launch
    test_power_cycle_installer
    test_update_livox_geph
    test_validate_livox_site_script
  )
  foreach(py_test IN LISTS LIVOX_RECOVERY_PY_TESTS)
    add_test(
      NAME ${py_test}
      COMMAND "${PYTHON_EXECUTABLE}"
              "${LIVOX_RECOVERY_QC_DIR}/${py_test}.py"
    )
    set_tests_properties(${py_test} PROPERTIES LABELS "livox_recovery_qc")
  endforeach()
endif()


#---------------------------------------------------------------------------------------
# Install
#---------------------------------------------------------------------------------------

install(TARGETS ${PROJECT_NAME}_node
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
	)

install(DIRECTORY
  launch
  config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
	)

#---------------------------------------------------------------------------------------
# end of CMakeList.txt
#---------------------------------------------------------------------------------------
