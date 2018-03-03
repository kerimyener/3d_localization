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

find_package( OpenMP )
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

find_package(CUDA)
if(CUDA_FOUND)
    add_definitions(-DCUDA_FOUND)
    list(APPEND FAST_PCL_PACKAGES ndt_gpu)
endif()


find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
  sensor_msgs
  pcl_conversions
  velodyne_pointcloud
  ${FAST_PCL_PACKAGES}
  ndt_cpu
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ndt_pcl fast_pcl
  CATKIN_DEPENDS std_msgs velodyne_pointcloud  ${FAST_PCL_PACKAGES}
 
#  DEPENDS ndt_tku
#  DEPENDS system_lib
)


###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS})

SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall ${CMAKE_CXX_FLAGS}")

add_executable(ndt_matching nodes/ndt_matching/ndt_matching.cpp)
add_executable(ndt_mapping_node nodes/ndt_mapping/ndt_mapping_node.cpp)

add_executable(voxel_grid_filter nodes/voxel_grid_filter/voxel_grid_filter.cpp)
target_link_libraries(ndt_matching ${catkin_LIBRARIES})
target_link_libraries(ndt_mapping_node ${catkin_LIBRARIES})


if(CUDA_FOUND)
  target_include_directories(ndt_matching PRIVATE ${CUDA_INCLUDE_DIRS})
  target_include_directories(ndt_mapping_node PRIVATE ${CUDA_INCLUDE_DIRS})
endif()


add_dependencies(voxel_grid_filter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(tf_mapping nodes/tf_mapping/tf_mapping.cpp)
add_executable(queue_counter nodes/queue_counter/queue_counter.cpp)
target_link_libraries(tf_mapping ${catkin_LIBRARIES})
target_link_libraries(queue_counter ${catkin_LIBRARIES})
target_link_libraries(voxel_grid_filter ${catkin_LIBRARIES})


