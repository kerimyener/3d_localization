#!/bin/bash
source /opt/ros/kinetic/setup.bash
#bash -c "/opt/ros/kinetic/bin/roscore"
gnome-terminal -e roscore
sleep 3s

# launch these nodes
source ~/master_ws/devel/setup.bash
rosparam set use_sim_time true
roslaunch nodes tf_localizer.launch &
roslaunch nodes state_publisher.launch &       
roslaunch nodes map_publisher.launch &

sleep 10s

#roslaunch nodes tf_world_map.launch &
roslaunch nodes voxel_filter.launch &
#roslaunch nodes odometry.launch &
roslaunch nodes ekf_wheel_imu.launch &
#roslaunch nodes navsat_transform_template.launch &

roslaunch nodes ndt_matching.launch &
#roslaunch nodes ekf_wheel_imu_ndt.launch &
#roslaunch nodes icp_matching.launch &
rviz&
roslaunch nodes bag_player.launch



