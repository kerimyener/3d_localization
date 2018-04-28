#!/bin/bash
gnome-terminal -e roscore
sleep 3s

# launch these nodes
rosparam set use_sim_time true
roslaunch nodes tf_localizer.launch &       
roslaunch nodes map_publisher.launch &
sleep 45s
roslaunch nodes tf_world_map.launch &
roslaunch nodes voxel_filter.launch &
roslaunch nodes ndt_matching.launch &
roslaunch nodes bag_player.launch 
rosnode kill points_map_loader
rosnode kill worl_to_map
rosnide kill voxel_grid_filter
rosnode kill base_link_to_localizer
rosnode kill ndt_matching


