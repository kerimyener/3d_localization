cmake_minimum_required(VERSION 2.8.3)
project(points_downsampler)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
  sensor_msgs
  pcl_conversions
  velodyne_pointcloud
  message_generation
  dynamic_reconfigure
)
find_package(Boost REQUIRED COMPONENTS system)

add_message_files(
  FILES
  PointsDownsamplerInfo.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)
generate_dynamic_reconfigure_options(
  cfg/DynCfgVoxel.cfg
)
catkin_package(
  CATKIN_DEPENDS sensor_msgs
)

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS}
                     SYSTEM ${catkin_INCLUDE_DIRS})
SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall ${CMAKE_CXX_FLAGS}")

add_executable(voxel_grid_filter nodes/voxel_grid_filter/voxel_grid_filter.cpp)

add_dependencies(voxel_grid_filter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(voxel_grid_filter ${PROJECT_NAME}_gencfg)

target_link_libraries(voxel_grid_filter ${catkin_LIBRARIES})

