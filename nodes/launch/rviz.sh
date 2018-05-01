#!/bin/bash
source /opt/ros/kinetic/setup.bash
source ~/master_ws/devel/setup.bash
rosrun rviz rviz  args="/home/kyyurtdas/master_ws/src/3d_localization/nodes/config/localization.rviz" &
roslaunch nodes state_publisher.launch
