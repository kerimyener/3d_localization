#ifndef NDT_MAPPING_HH_

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <velodyne_pointcloud/point_types.h>
#include <velodyne_pointcloud/rawdata.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#ifdef USE_FAST_PCL
#include <fast_pcl/filters/voxel_grid.h>
#include <fast_pcl/registration/ndt.h>
#else
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/ndt.h>
#endif


#ifdef CUDA_FOUND
#include <fast_pcl/ndt_gpu/NormalDistributionsTransform.h>
#endif

#include <fast_pcl/ndt_cpu/NormalDistributionsTransform.h>

#include <time.h>
namespace ndt {
class NDT_MAPP{
public:

  void setup(ros::NodeHandle& nh,ros::NodeHandle& private_nh);
  void output_callback(const std_msgs::Bool& input);
  void imu_odom_calc(ros::Time current_time);
  void odom_calc(ros::Time current_time);
  void imu_calc(ros::Time current_time);
  double wrapToPm(double a_num, const double a_max);
  double wrapToPmPi(double a_angle_rad);
  void odom_callback(const nav_msgs::Odometry::ConstPtr& input);
  void imuUpsideDown(const sensor_msgs::Imu::Ptr input);
  void imu_callback(const sensor_msgs::Imu::Ptr& input);
  void points_callback(const sensor_msgs::PointCloud2::ConstPtr& input);

private:
  struct pose{
    double x;
    double y;
    double z;
    double roll;
    double pitch;
    double yaw;
  };
  pose previous_pose, guess_pose, guess_pose_imu, guess_pose_odom, guess_pose_imu_odom, 		current_pose,
  current_pose_imu, current_pose_odom, current_pose_imu_odom, ndt_pose, added_pose, 		localizer_pose;

  ros::Time current_scan_time;
  ros::Time previous_scan_time;
  ros::Duration scan_duration;

  double diff = 0.0;
  double diff_x = 0.0, diff_y = 0.0, diff_z = 0.0, diff_yaw;  // current_pose - 		previous_pose
  double offset_imu_x, offset_imu_y, offset_imu_z, offset_imu_roll, offset_imu_pitch, 		offset_imu_yaw;
  double offset_odom_x, offset_odom_y, offset_odom_z, offset_odom_roll, 			offset_odom_pitch, offset_odom_yaw;
  double offset_imu_odom_x, offset_imu_odom_y, offset_imu_odom_z, offset_imu_odom_roll, 		offset_imu_odom_pitch,offset_imu_odom_yaw;

  double current_velocity_x = 0.0;
  double current_velocity_y = 0.0;
  double current_velocity_z = 0.0;

  double current_velocity_imu_x = 0.0;
  double current_velocity_imu_y = 0.0;
  double current_velocity_imu_z = 0.0;

  pcl::PointCloud<pcl::PointXYZI> map;

  // Added for GPU ndt
#ifdef CUDA_FOUND
  gpu::GNormalDistributionsTransform gpu_ndt;
#endif

  // Added for CPU ndt testing version
  cpu::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> cpu_ndt;

  pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt;
  // end of adding

  // Default values
  int max_iter = 30;        // Maximum iterations
  float ndt_res = 1.0;      // Resolution
  double step_size = 0.1;   // Step size
  double trans_eps = 0.01;  // Transformation epsilon

  // Leaf size of VoxelGrid filter.
  double voxel_leaf_size = 2.0;


  geometry_msgs::PoseStamped current_pose_msg, guess_pose_msg;

  ros::Publisher ndt_stat_pub;
  std_msgs::Bool ndt_stat_msg;

  int initial_scan_loaded = 0;

  Eigen::Matrix4f gnss_transform = Eigen::Matrix4f::Identity();

  double min_scan_range = 5.0;
  double min_add_scan_shift = 1.0;

  double _tf_x, _tf_y, _tf_z, _tf_roll, _tf_pitch, _tf_yaw;
  Eigen::Matrix4f tf_btol, tf_ltob;

  bool isMapUpdate = true;

  bool _use_openmp = false;
  bool _use_gpu = false;

  bool _use_fast_pcl = false;

  bool _use_imu = false;
  bool _use_odom = false;
  bool _imu_upside_down = false;

  std::string _imu_topic = "/imu_raw";
  std::string _velodyne_topic = "/velodyne/velodyne_points";
  double fitness_score;
  bool has_converged;
  int final_num_iteration;

  sensor_msgs::Imu imu;

  nav_msgs::Odometry odom;
  ros::Subscriber output_sub ;
  ros::Subscriber points_sub ;
  ros::Subscriber odom_sub ;
  ros::Subscriber imu_sub ;

  ros::Time callback_start, callback_end, t1_start, t1_end, t2_start, t2_end, t3_start, t3_end, t4_start, t4_end,
  t5_start, t5_end;
  ros::Duration d_callback, d1, d2, d3, d4, d5;

  ros::Publisher ndt_map_pub;
  ros::Publisher current_pose_pub;
  ros::Publisher guess_pose_linaer_pub;

};
}
#endif
