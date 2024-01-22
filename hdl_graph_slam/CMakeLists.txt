# SPDX-License-Identifier: BSD-2-Clause
cmake_minimum_required(VERSION 2.8.3)
project(hdl_graph_slam)

# Can we use C++17 in indigo?
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  add_definitions(-std=c++11)
  set(CMAKE_CXX_FLAGS "-std=c++11")
else()
  add_definitions(-std=c++14 -msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2)
  set(CMAKE_CXX_FLAGS "-std=c++14 -msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2")
endif()

include(cmake/package.cmake)

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")
# pcl 1.7 causes a segfault when it is built with debug mode
set(CMAKE_BUILD_TYPE "RELEASE")

catkin_python_setup()

########################
## message generation ##
########################
add_message_files(FILES
  FloorCoeffs.msg
  ScanMatchingStatus.msg
  CustomMsg.msg
  CustomPoint.msg
  Pose6D.msg
)

add_service_files(FILES
  SaveMap.srv
  LoadGraph.srv
  DumpGraph.srv
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES hdl_graph_slam_nodelet
  CATKIN_DEPENDS
    geometry_msgs
    message_runtime
    nmea_msgs
    roscpp
    sensor_msgs
    std_msgs
    tf_conversions
#  DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(/home/idriver/slam_dai/hdl_ws/devel/include)
include_directories(include)
include_directories(
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  src
)

# nodelets
add_library(prefiltering_nodelet apps/prefiltering_nodelet.cpp)
target_link_libraries(prefiltering_nodelet
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)


add_library(floor_detection_nodelet apps/floor_detection_nodelet.cpp)
target_link_libraries(floor_detection_nodelet
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
add_dependencies(floor_detection_nodelet ${PROJECT_NAME}_gencpp)


add_library(scan_matching_odometry_nodelet
  apps/scan_matching_odometry_nodelet.cpp
  src/hdl_graph_slam/registrations.cpp
)
target_link_libraries(scan_matching_odometry_nodelet
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
add_dependencies(scan_matching_odometry_nodelet ${PROJECT_NAME}_gencpp)


add_library(hdl_graph_slam_nodelet
  apps/hdl_graph_slam_nodelet.cpp
  src/hdl_graph_slam/graph_slam.cpp
  src/hdl_graph_slam/keyframe.cpp
  src/hdl_graph_slam/map_cloud_generator.cpp
  src/hdl_graph_slam/registrations.cpp
  src/hdl_graph_slam/information_matrix_calculator.cpp
  src/g2o/robust_kernel_io.cpp
)
target_link_libraries(hdl_graph_slam_nodelet
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${G2O_LIBS}
)
add_dependencies(hdl_graph_slam_nodelet ${PROJECT_NAME}_gencpp)

add_library(hdl_lio_nodelet
        apps/front_lio_nodelet.cpp
        src/preprocess.cpp
        include/ikd-Tree/ikd_Tree.cpp
        )
link_libraries(hdl_lio_nodelet
        ${catkin_LIBRARIES} ${PCL_LIBRARIES}
        )

catkin_install_python(
  PROGRAMS
    src/${PROJECT_NAME}/bag_player.py
    src/${PROJECT_NAME}/ford2bag.py
    src/${PROJECT_NAME}/map2odom_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS
  prefiltering_nodelet
  floor_detection_nodelet
  scan_matching_odometry_nodelet
  hdl_graph_slam_nodelet
  hdl_lio_nodelet
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/
   DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)
