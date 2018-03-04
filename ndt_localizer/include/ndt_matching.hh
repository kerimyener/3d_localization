#ifndef NDT_MATCHING_HH_

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

#ifdef USE_FAST_PCL
#include <fast_pcl/registration/ndt.h>
#else
#include <pcl/registration/ndt.h>
#endif

#ifdef CUDA_FOUND
#include <fast_pcl/ndt_gpu/NormalDistributionsTransform.h>
#endif

//End of adding

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>


//Added for testing on cpu
#include <fast_pcl/ndt_cpu/NormalDistributionsTransform.h>
//End of adding

#define PREDICT_POSE_THRESHOLD 0.5

#define Wa 0.4
#define Wb 0.3
#define Wc 0.3

namespace ndt{

class NDT_MATCH{
public:
  void setup(ros::NodeHandle& nh,ros::NodeHandle& private_nh);
  void map_callback(const sensor_msgs::PointCloud2::ConstPtr& input);
  void gnss_callback(const geometry_msgs::PoseStamped::ConstPtr& input);
  void initialpose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& input);
  void imu_odom_calc(ros::Time current_time);
  void odom_calc(ros::Time current_time);
  void imu_calc(ros::Time current_time);
  const double wrapToPm(double a_num, const double a_max);
  const double wrapToPmPi(double a_angle_rad);
  void odom_callback(const nav_msgs::Odometry::ConstPtr& input);
  void imuUpsideDown(const sensor_msgs::Imu::Ptr input);
  void imu_callback(const sensor_msgs::Imu::Ptr& input);
  void points_callback(const sensor_msgs::PointCloud2::ConstPtr& input);

private:
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
      ndt_pose, current_pose, current_pose_imu, current_pose_odom, current_pose_imu_odom, localizer_pose,
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
   int _use_gnss = 0;
   int init_pos_set = 1;

  #ifdef CUDA_FOUND
   std::shared_ptr<gpu::GNormalDistributionsTransform> gpu_ndt_ptr = std::make_shared<gpu::GNormalDistributionsTransform>();
  #endif


   cpu::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> cpu_ndt;

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
   geometry_msgs::PoseStamped ndt_pose_msg;

  // current_pose is published by vel_pose_mux
  /*
   ros::Publisher current_pose_pub;
   geometry_msgs::PoseStamped current_pose_msg;
  */

   ros::Publisher localizer_pose_pub;
   geometry_msgs::PoseStamped localizer_pose_msg;

   ros::Publisher estimate_twist_pub;
   geometry_msgs::TwistStamped estimate_twist_msg;

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

   ros::Publisher estimated_vel_mps_pub, estimated_vel_kmph_pub, estimated_vel_pub;
   std_msgs::Float32 estimated_vel_mps, estimated_vel_kmph, previous_estimated_vel_kmph;

   std::chrono::time_point<std::chrono::system_clock> matching_start, matching_end;

   ros::Publisher time_ndt_matching_pub;
   std_msgs::Float32 time_ndt_matching;

   int _queue_size = 1000;

   ros::Publisher ndt_stat_pub;
  // autoware_msgs::ndt_stat ndt_stat_msg;

   double predict_pose_error = 0.0;

   double _tf_x, _tf_y, _tf_z, _tf_roll, _tf_pitch, _tf_yaw;
   Eigen::Matrix4f tf_btol;

   std::string _localizer = "velodyne";
   std::string _offset = "linear";  // linear, zero, quadratic

   ros::Publisher ndt_reliability_pub;
   std_msgs::Float32 ndt_reliability;

   bool _use_gpu = false;
   bool _use_openmp = false;

   bool _use_fast_pcl = false;

   bool _get_height = false;
   bool _use_local_transform = false;
   bool _use_imu = false;
   bool _use_odom = false;
   bool _imu_upside_down = false;

   std::string _imu_topic = "/imu_raw";

   std::ofstream ofs;
   std::string filename;

   sensor_msgs::Imu imu;
   nav_msgs::Odometry odom;

  //  tf::TransformListener local_transform_listener;
   tf::StampedTransform local_transform;

   int points_map_num = 0;

  pthread_mutex_t mutex;
  // Subscribers
  ros::Subscriber param_sub;
  ros::Subscriber gnss_sub;
  ros::Subscriber map_sub;
  ros::Subscriber initialpose_sub;
  ros::Subscriber points_sub;
  ros::Subscriber odom_sub;
  ros::Subscriber imu_sub;


};
}

#endif