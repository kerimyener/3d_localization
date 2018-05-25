cmake_minimum_required(VERSION 2.8.3)
project(ndt_localizer)
execute_process(
  COMMAND rosversion -d
  OUTPUT_VARIABLE ROS_VERSION
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

find_package(PCL REQUIRED)

IF(NOT (PCL_VERSION VERSION_LESS "1.7.2"))
SET(FAST_PCL_PACKAGES filters registration)
ENDIF(NOT (PCL_VERSION VERSION_LESS "1.7.2"))

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
  sensor_msgs
  pcl_conversions
  velodyne_pointcloud
  ${FAST_PCL_PACKAGES}
  ndt_cpu 
  localization_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ndt_pcl
  CATKIN_DEPENDS std_msgs velodyne_pointcloud ${FAST_PCL_PACKAGES} localization_msgs
  DEPENDS ndt_cpu
#  DEPENDS ndt_tku
#  DEPENDS system_lib
)


###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS})

SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall ${CMAKE_CXX_FLAGS}")

add_executable(ndt_matching nodes/ndt_matching/ndt_matching.cpp)
add_executable(ndt_dnm nodes/ndt_matching/common.cc nodes/ndt_matching/ndt_node.cpp)

add_executable(ndt_mapping nodes/ndt_mapping/ndt_mapping.cpp)
target_link_libraries(ndt_matching ${catkin_LIBRARIES})
target_link_libraries(ndt_dnm ${catkin_LIBRARIES})

target_link_libraries(ndt_mapping ${catkin_LIBRARIES})
add_dependencies(ndt_matching localization_msgs_generate_messages_cpp)




add_executable(tf_mapping nodes/tf_mapping/tf_mapping.cpp)

add_executable(queue_counter nodes/queue_counter/queue_counter.cpp)

target_link_libraries(tf_mapping ${catkin_LIBRARIES})

target_link_libraries(queue_counter ${catkin_LIBRARIES})

install(TARGETS ndt_matching
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)




