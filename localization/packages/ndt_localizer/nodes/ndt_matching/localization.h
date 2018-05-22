#ifndef LOCALIZATION_H_
#define LOCALIZATION_H_
#include <chrono>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <memory>
#include <pthread.h>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>

#include <velodyne_pointcloud/point_types.h>
#include <velodyne_pointcloud/rawdata.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

//Added for testing on cpu
#include <fast_pcl/ndt_cpu/NormalDistributionsTransform.h>
//End of adding

#include "localization_msgs/ndt_stat.h"
#include "localization_msgs/icp_stat.h"



#define PREDICT_POSE_THRESHOLD 0.5

#define Wa 0.4
#define Wb 0.3
#define Wc 0.3

namespace localizer{

class core{
public:
  void param_callback(ros::NodeHandle nh);
  void map_callback(const sensor_msgs::PointCloud2::ConstPtr& input);
  void gnss_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& input);
  void initialpose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& input);
  void imu_callback(const sensor_msgs::Imu::Ptr& input);
  void odom_callback(const nav_msgs::Odometry::ConstPtr& input);
  void points_callback(const sensor_msgs::PointCloud2::ConstPtr& input);
  void imu_odom_calc(ros::Time current_time);
  void odom_calc(ros::Time current_time);
  void imu_calc(ros::Time current_time);
  const double wrapToPm(double a_num, const double a_max);
  const double wrapToPmPi(double a_angle_rad);
  void imuUpsideDown(const sensor_msgs::Imu::Ptr input);
  void odomPub (const ros::TimerEvent& event);
  void toInitNdt();
  void toInitIcp();

private:
  int init_pos_gnss;
  float pose_x;
  float pose_y;
  float pose_z;
  float pose_roll;
  float pose_pitch;
  float pose_yaw;
  float resolution;
  float stepsize;
  float trans_epsilon;
  int max_iterations;
  int _maximum_iterations = 100;
  double _transformation_epsilon = 0.01;
  double _max_correspondence_distance = 1.0;
  double _euclidean_fitness_epsilon = 0.1;
  double _ransac_outlier_rejection_threshold = 1.0;

  pthread_mutex_t mutex;



  struct pose
  {
    double x;
    double y;
    double z;
    double roll;
    double pitch;
    double yaw;
  };

  pose initial_pose, predict_pose, predict_pose_imu, predict_pose_odom, predict_pose_imu_odom, previous_pose,
  ndt_pose, icp_pose, current_pose, current_pose_imu, current_pose_odom, current_pose_imu_odom, localizer_pose,
  previous_gnss_pose, current_gnss_pose;

  double offset_x, offset_y, offset_z, offset_yaw;  // current_pos - previous_pose
  double offset_imu_x, offset_imu_y, offset_imu_z, offset_imu_roll, offset_imu_pitch, offset_imu_yaw;
  double offset_odom_x, offset_odom_y, offset_odom_z, offset_odom_roll, offset_odom_pitch, offset_odom_yaw;
  double offset_imu_odom_x, offset_imu_odom_y, offset_imu_odom_z, offset_imu_odom_roll, offset_imu_odom_pitch,
  offset_imu_odom_yaw;

  // Can't load if typed "pcl::PointCloud<pcl::PointXYZRGB> map, add;"
  pcl::PointCloud<pcl::PointXYZ> map, add;

  // If the map is loaded, map_loaded will be 1.
  int map_loaded = 0;
  int _use_gnss = 1;
  int init_pos_set = 0;

  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;

  // Default values

  int max_iter = 30;        // Maximum iterations
  float ndt_res = 1.0;      // Resolution
  double step_size = 0.1;   // Step size
  double trans_eps = 0.01;  // Transformation epsilon

  ros::Publisher predict_pose_pub;
  geometry_msgs::PoseStamped predict_pose_msg;

  ros::Publisher predict_pose_imu_pub;
  geometry_msgs::PoseStamped predict_pose_imu_msg;

  ros::Publisher predict_pose_odom_pub;
  geometry_msgs::PoseStamped predict_pose_odom_msg;

  ros::Publisher predict_pose_imu_odom_pub;
  geometry_msgs::PoseStamped predict_pose_imu_odom_msg;

  ros::Publisher ndt_pose_pub;
  geometry_msgs::PoseWithCovarianceStamped ndt_pose_msg;

  ros::Publisher icp_pose_pub;
  geometry_msgs::PoseWithCovarianceStamped icp_pose_msg;

  // current_pose is published by vel_pose_mux
  /*
   ros::Publisher current_pose_pub;
   geometry_msgs::PoseStamped current_pose_msg;
  */

  ros::Publisher localizer_pose_pub;
  geometry_msgs::PoseStamped localizer_pose_msg;

  ros::Publisher estimate_twist_pub;
  geometry_msgs::TwistStamped estimate_twist_msg;

  ros::Publisher ndt_reliability_pub;
  std_msgs::Float32 ndt_reliability;

  ros::Publisher estimated_vel_mps_pub, estimated_vel_kmph_pub, estimated_vel_pub;
  std_msgs::Float32 estimated_vel_mps, estimated_vel_kmph, previous_estimated_vel_kmph;

  ros::Publisher time_ndt_matching_pub;
  std_msgs::Float32 time_ndt_matching;

  ros::Publisher ndt_stat_pub;
  localization_msgs::ndt_stat ndt_stat_msg;

  ros::Publisher icp_stat_pub;
  localization_msgs::icp_stat icp_stat_msg;

  ros::Publisher odom_pub;
  nav_msgs::Odometry odom;
  sensor_msgs::Imu imu;

  ros::Time current_scan_time;
  ros::Time previous_scan_time;
  ros::Duration scan_duration;

  double exe_time = 0.0;
  bool has_converged;
  int iteration = 0;
  double fitness_score = 0.0;
  double trans_probability = 0.0;

  double diff = 0.0;
  double diff_x = 0.0, diff_y = 0.0, diff_z = 0.0, diff_yaw;

  double current_velocity = 0.0, previous_velocity = 0.0, previous_previous_velocity = 0.0;  // [m/s]
  double current_velocity_x = 0.0, previous_velocity_x = 0.0;
  double current_velocity_y = 0.0, previous_velocity_y = 0.0;
  double current_velocity_z = 0.0, previous_velocity_z = 0.0;
  //  double current_velocity_yaw = 0.0, previous_velocity_yaw = 0.0;
  double current_velocity_smooth = 0.0;

  double current_velocity_imu_x = 0.0;
  double current_velocity_imu_y = 0.0;
  double current_velocity_imu_z = 0.0;

  double current_accel = 0.0, previous_accel = 0.0;  // [m/s^2]
  double current_accel_x = 0.0;
  double current_accel_y = 0.0;
  double current_accel_z = 0.0;
  //  double current_accel_yaw = 0.0;

  double angular_velocity = 0.0;

  int use_predict_pose = 0;

  std::chrono::time_point<std::chrono::system_clock> matching_start, matching_end;

  int _queue_size = 1000;

  double predict_pose_error = 0.0;

  double _tf_x, _tf_y, _tf_z, _tf_roll, _tf_pitch, _tf_yaw;
  Eigen::Matrix4f tf_btol;

  std::string _localizer = "velodyne";
  std::string _offset = "linear";  // linear, zero, quadratic

  bool _get_height = false;
  bool _use_imu = false;
  bool _use_odom = false;
  bool _imu_upside_down = false;

  std::string _imu_topic = "/imu_raw";

  std::ofstream ofs;
  std::string filename;
  //  tf::TransformListener local_transform_listener;
  tf::StampedTransform local_transform;

  uint points_map_num = 0;

};
}



#endif 
