#include "localiser_core.h"

namespace localiser {
core::core(){
    pthread_mutex_init(&mutex, NULL);
    initForROS();

}

core::~core(){
}

void core::initForROS(){

    nh.getParam("initial_pos_gnss",init_pos_gnss);
    nh.getParam("x_pose",pose_x);
    nh.getParam("y_pose",pose_y);
    nh.getParam("z_pose",pose_z);
    nh.getParam("roll_pose",pose_roll);
    nh.getParam("pitch_pose",pose_pitch);
    nh.getParam("yaw_pose",pose_yaw);
    nh.getParam("predict_pose_use",use_predict_pose);
    nh.getParam("resolution",resolution);
    nh.getParam("step_size",stepsize);
    nh.getParam("transepsilon",trans_epsilon);
    nh.getParam("maxiterations",max_iterations);
    private_nh.getParam("use_gnss", _use_gnss);
    private_nh.getParam("queue_size", _queue_size);
    private_nh.getParam("use_imu", _use_imu);
    private_nh.getParam("use_odom", _use_odom);
    private_nh.getParam("imu_upside_down", _imu_upside_down);
    private_nh.getParam("imu_topic", _imu_topic);

    if (nh.getParam("localizer", _localizer) == false)
    {
      std::cout << "localizer is not set." << std::endl;
      return ;
    }
    if (nh.getParam("tf_x", _tf_x) == false)
    {
      std::cout << "tf_x is not set." << std::endl;
      return ;
    }
    if (nh.getParam("tf_y", _tf_y) == false)
    {
      std::cout << "tf_y is not set." << std::endl;
      return ;
    }
    if (nh.getParam("tf_z", _tf_z) == false)
    {
      std::cout << "tf_z is not set." << std::endl;
      return ;
    }
    if (nh.getParam("tf_roll", _tf_roll) == false)
    {
      std::cout << "tf_roll is not set." << std::endl;
      return ;
    }
    if (nh.getParam("tf_pitch", _tf_pitch) == false)
    {
      std::cout << "tf_pitch is not set." << std::endl;
      return ;
    }
    if (nh.getParam("tf_yaw", _tf_yaw) == false)
    {
      std::cout << "tf_yaw is not set." << std::endl;
      return ;
    }

    if (_use_gnss != init_pos_gnss)
    {
      init_pos_set = 0;
    }
    else if (_use_gnss == 0 &&
             (initial_pose.x != pose_x || initial_pose.y != pose_y || initial_pose.z != pose_z ||
              initial_pose.roll != pose_roll || initial_pose.pitch != pose_pitch || initial_pose.yaw != pose_yaw))
    {
      init_pos_set = 0;
    }

    _use_gnss = init_pos_gnss;

    // Setting parameters
    if (resolution != ndt_res)
    {
      ndt_res = resolution;
      ndt.setResolution(ndt_res);
    }
    if (stepsize != step_size)
    {
      step_size = stepsize;
      ndt.setStepSize(step_size);

    }
    if (trans_epsilon != trans_eps)
    {
      trans_eps = trans_epsilon;
      ndt.setTransformationEpsilon(trans_eps);

    }
    if (max_iterations != max_iter)
    {
      max_iter = max_iterations;
      ndt.setMaximumIterations(max_iter);

    }

    if (_use_gnss == 0 && init_pos_set == 0)
    {
      initial_pose.x = pose_x;
      initial_pose.y = pose_y;
      initial_pose.z = pose_z;
      initial_pose.roll = pose_roll;
      initial_pose.pitch = pose_pitch;
      initial_pose.yaw = pose_yaw;


      // Setting position and posture for the first time.
      localizer_pose.x = initial_pose.x;
      localizer_pose.y = initial_pose.y;
      localizer_pose.z = initial_pose.z;
      localizer_pose.roll = initial_pose.roll;
      localizer_pose.pitch = initial_pose.pitch;
      localizer_pose.yaw = initial_pose.yaw;

      previous_pose.x = initial_pose.x;
      previous_pose.y = initial_pose.y;
      previous_pose.z = initial_pose.z;
      previous_pose.roll = initial_pose.roll;
      previous_pose.pitch = initial_pose.pitch;
      previous_pose.yaw = initial_pose.yaw;

      current_pose.x = initial_pose.x;
      current_pose.y = initial_pose.y;
      current_pose.z = initial_pose.z;
      current_pose.roll = initial_pose.roll;
      current_pose.pitch = initial_pose.pitch;
      current_pose.yaw = initial_pose.yaw;

      current_velocity = 0;
      current_velocity_x = 0;
      current_velocity_y = 0;
      current_velocity_z = 0;
      angular_velocity = 0;

      current_pose_imu.x = 0;
      current_pose_imu.y = 0;
      current_pose_imu.z = 0;
      current_pose_imu.roll = 0;
      current_pose_imu.pitch = 0;
      current_pose_imu.yaw = 0;

      current_velocity_imu_x = current_velocity_x;
      current_velocity_imu_y = current_velocity_y;
      current_velocity_imu_z = current_velocity_z;
      init_pos_set = 1;
    }


    char buffer[80];
    std::time_t now = std::time(NULL);
    std::tm* pnow = std::localtime(&now);
    std::strftime(buffer, 80, "%Y%m%d_%H%M%S", pnow);
    filename = "ndt_matching_" + std::string(buffer) + ".csv";
    ofs.open(filename.c_str(), std::ios::app);

    std::cout << "-----------------------------------------------------------------" << std::endl;
    std::cout << "Log file: " << filename << std::endl;
    std::cout << "use_gnss: " << _use_gnss << std::endl;
    std::cout << "queue_size: " << _queue_size << std::endl;
    std::cout << "offset: " << _offset << std::endl;
    std::cout << "get_height: " << _get_height << std::endl;
    std::cout << "use_imu: " << _use_imu << std::endl;
    std::cout << "use_odom: " << _use_odom << std::endl;
    std::cout << "imu_upside_down: " << _imu_upside_down << std::endl;
    std::cout << "localizer: " << _localizer << std::endl;
    std::cout << "imu_topic: " << _imu_topic << std::endl;
    std::cout << "(tf_x,tf_y,tf_z,tf_roll,tf_pitch,tf_yaw): (" << _tf_x << ", " << _tf_y << ", " << _tf_z << ", "
              << _tf_roll << ", " << _tf_pitch << ", " << _tf_yaw << ")" << std::endl;
    std::cout << "-----------------------------------------------------------------" << std::endl;

    Eigen::Translation3f tl_btol(_tf_x, _tf_y, _tf_z);                 // tl: translation
    Eigen::AngleAxisf rot_x_btol(_tf_roll, Eigen::Vector3f::UnitX());  // rot: rotation
    Eigen::AngleAxisf rot_y_btol(_tf_pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf rot_z_btol(_tf_yaw, Eigen::Vector3f::UnitZ());
    tf_btol = (tl_btol * rot_z_btol * rot_y_btol * rot_x_btol).matrix();

    // Updated in initialpose_callback or gnss_callback
    initial_pose.x = 0.0;
    initial_pose.y = 0.0;
    initial_pose.z = 0.0;
    initial_pose.roll = 0.0;
    initial_pose.pitch = 0.0;
    initial_pose.yaw = 0.0;

    predict_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/predict_pose", 10);
    predict_pose_imu_pub = nh.advertise<geometry_msgs::PoseStamped>("/predict_pose_imu", 10);
    predict_pose_odom_pub = nh.advertise<geometry_msgs::PoseStamped>("/predict_pose_odom", 10);
    predict_pose_imu_odom_pub = nh.advertise<geometry_msgs::PoseStamped>("/predict_pose_imu_odom", 10);
    ndt_pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/ndt_pose", 10);
    // current_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/current_pose", 10);
    localizer_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/localizer_pose", 10);
    estimate_twist_pub = nh.advertise<geometry_msgs::TwistStamped>("/estimate_twist", 10);
    estimated_vel_mps_pub = nh.advertise<std_msgs::Float32>("/estimated_vel_mps", 10);
    estimated_vel_kmph_pub = nh.advertise<std_msgs::Float32>("/estimated_vel_kmph", 10);
    estimated_vel_pub = nh.advertise<geometry_msgs::Vector3Stamped>("/estimated_vel", 10);
    time_ndt_matching_pub = nh.advertise<std_msgs::Float32>("/time_ndt_matching", 10);
    ndt_stat_pub = nh.advertise<localization_msgs::ndt_stat>("/ndt_stat", 10);
    ndt_reliability_pub = nh.advertise<std_msgs::Float32>("/ndt_reliability", 10);
    odom_pub = nh.advertise<nav_msgs::Odometry>("odometry/laser", 10);
    // Subscribers
    gnss_sub = nh.subscribe("gnss_pose", 10, &core::gnss_callback, this);
    map_sub = nh.subscribe("points_map", 1, &core::map_callback, this);
    initialpose_sub = nh.subscribe("initialpose", 10, &core::initialpose_callback, this);
    points_sub = nh.subscribe("filtered_points", _queue_size, &core::points_callback, this);
    odom_sub = nh.subscribe("/odom_pose", _queue_size * 10, &core::odom_callback, this);
    imu_sub = nh.subscribe(_imu_topic.c_str(), _queue_size * 10, &core::imu_callback, this);
}

void core::run(){
  ros::spin();
}

void core::map_callback(const sensor_msgs::PointCloud2::ConstPtr &input){
    // if (map_loaded == 0)
    if (points_map_num != input->width)
    {
      std::cout << "Update points_map." << std::endl;

      points_map_num = input->width;

      // Convert the data type(from sensor_msgs to pcl).
      pcl::fromROSMsg(*input, map);

      pcl::PointCloud<pcl::PointXYZ>::Ptr map_ptr(new pcl::PointCloud<pcl::PointXYZ>(map));


      // Setting point cloud to be aligned to.

      pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> new_ndt;
      pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
      new_ndt.setResolution(ndt_res);
      new_ndt.setInputTarget(map_ptr);
      new_ndt.setMaximumIterations(max_iter);
      new_ndt.setStepSize(step_size);
      new_ndt.setTransformationEpsilon(trans_eps);
      new_ndt.align(*output_cloud, Eigen::Matrix4f::Identity());
      pthread_mutex_lock(&mutex);
      ndt = new_ndt;
      pthread_mutex_unlock(&mutex);

      map_loaded = 1;
    }
}
void core::odomPub(ros::Time current, double x, double y, double yaw, double vx, double vy, double vz){
    tf::TransformBroadcaster odom_broadcaster;
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(yaw);

    //first, we'll publish the transform over tf
    /*geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current;
    odom_trans.header.frame_id = "map";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = current_pose.x;
    odom_trans.transform.translation.y = current_pose.y;
    odom_trans.transform.translation.z = current_pose.z;
    odom_trans.transform.rotation = odom_quat;
    odom_broadcaster.sendTransform(odom_trans);*/

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current;
    odom.header.frame_id = "map";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vz;

    //publish the message
    odom_pub.publish(odom);
}

void core::gnss_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &input){
    tf::Quaternion gnss_q(input->pose.pose.orientation.x, input->pose.pose.orientation.y, input->pose.pose.orientation.z,
                          input->pose.pose.orientation.w);
    tf::Matrix3x3 gnss_m(gnss_q);
    current_gnss_pose.x = input->pose.pose.position.x;
    current_gnss_pose.y = input->pose.pose.position.y;
    current_gnss_pose.z = input->pose.pose.position.z;
    gnss_m.getRPY(current_gnss_pose.roll, current_gnss_pose.pitch, current_gnss_pose.yaw);

    if ((_use_gnss == 1 && init_pos_set == 0) || fitness_score >= 500.0)
    {
      previous_pose.x = previous_gnss_pose.x;
      previous_pose.y = previous_gnss_pose.y;
      previous_pose.z = previous_gnss_pose.z;
      previous_pose.roll = previous_gnss_pose.roll;
      previous_pose.pitch = previous_gnss_pose.pitch;
      previous_pose.yaw = previous_gnss_pose.yaw;

      current_pose.x = current_gnss_pose.x;
      current_pose.y = current_gnss_pose.y;
      current_pose.z = current_gnss_pose.z;
      current_pose.roll = current_gnss_pose.roll;
      current_pose.pitch = current_gnss_pose.pitch;
      current_pose.yaw = current_gnss_pose.yaw;

      current_pose_imu = current_pose_odom = current_pose_imu_odom = current_pose;

      offset_x = current_pose.x - previous_pose.x;
      offset_y = current_pose.y - previous_pose.y;
      offset_z = current_pose.z - previous_pose.z;
      offset_yaw = current_pose.yaw - previous_pose.yaw;

      init_pos_set = 1;
    }

    previous_gnss_pose.x = current_gnss_pose.x;
    previous_gnss_pose.y = current_gnss_pose.y;
    previous_gnss_pose.z = current_gnss_pose.z;
    previous_gnss_pose.roll = current_gnss_pose.roll;
    previous_gnss_pose.pitch = current_gnss_pose.pitch;
    previous_gnss_pose.yaw = current_gnss_pose.yaw;
}

void core::initialpose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &input){
    tf::TransformListener listener;
    tf::StampedTransform transform;
    try
    {
      ros::Time now = ros::Time(0);
      listener.waitForTransform("/map", input->header.frame_id, now, ros::Duration(10.0));
      listener.lookupTransform("/map", input->header.frame_id, now, transform);
    }
    catch (tf::TransformException& ex)
    {
      ROS_ERROR("%s", ex.what());
    }

    tf::Quaternion q(input->pose.pose.orientation.x, input->pose.pose.orientation.y, input->pose.pose.orientation.z,
                     input->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);

    current_pose.x = input->pose.pose.position.x + transform.getOrigin().x();
    current_pose.y = input->pose.pose.position.y + transform.getOrigin().y();
    current_pose.z = input->pose.pose.position.z + transform.getOrigin().z();

    m.getRPY(current_pose.roll, current_pose.pitch, current_pose.yaw);

    if (_get_height == true && map_loaded == 1)
    {
      double min_distance = DBL_MAX;
      double nearest_z = current_pose.z;
      for (const auto& p : map)
      {
        double distance = hypot(current_pose.x - p.x, current_pose.y - p.y);
        if (distance < min_distance)
        {
          min_distance = distance;
          nearest_z = p.z;
        }
      }
      current_pose.z = nearest_z;
    }

    current_pose_imu = current_pose_odom = current_pose_imu_odom = current_pose;
    previous_pose.x = current_pose.x;
    previous_pose.y = current_pose.y;
    previous_pose.z = current_pose.z;
    previous_pose.roll = current_pose.roll;
    previous_pose.pitch = current_pose.pitch;
    previous_pose.yaw = current_pose.yaw;

    offset_x = 0.0;
    offset_y = 0.0;
    offset_z = 0.0;
    offset_yaw = 0.0;

    offset_imu_x = 0.0;
    offset_imu_y = 0.0;
    offset_imu_z = 0.0;
    offset_imu_roll = 0.0;
    offset_imu_pitch = 0.0;
    offset_imu_yaw = 0.0;

    offset_odom_x = 0.0;
    offset_odom_y = 0.0;
    offset_odom_z = 0.0;
    offset_odom_roll = 0.0;
    offset_odom_pitch = 0.0;
    offset_odom_yaw = 0.0;

    offset_imu_odom_x = 0.0;
    offset_imu_odom_y = 0.0;
    offset_imu_odom_z = 0.0;
    offset_imu_odom_roll = 0.0;
    offset_imu_odom_pitch = 0.0;
    offset_imu_odom_yaw = 0.0;
}

void core::imu_odom_calc(ros::Time current_time){
    static ros::Time previous_time = current_time;
    double diff_time = (current_time - previous_time).toSec();

    double diff_imu_roll = imu.angular_velocity.x * diff_time;
    double diff_imu_pitch = imu.angular_velocity.y * diff_time;
    double diff_imu_yaw = imu.angular_velocity.z * diff_time;

    current_pose_imu_odom.roll += diff_imu_roll;
    current_pose_imu_odom.pitch += diff_imu_pitch;
    current_pose_imu_odom.yaw += diff_imu_yaw;

    double diff_distance = odom.twist.twist.linear.x * diff_time;
    offset_imu_odom_x += diff_distance * cos(-current_pose_imu_odom.pitch) * cos(current_pose_imu_odom.yaw);
    offset_imu_odom_y += diff_distance * cos(-current_pose_imu_odom.pitch) * sin(current_pose_imu_odom.yaw);
    offset_imu_odom_z += diff_distance * sin(-current_pose_imu_odom.pitch);

    offset_imu_odom_roll += diff_imu_roll;
    offset_imu_odom_pitch += diff_imu_pitch;
    offset_imu_odom_yaw += diff_imu_yaw;

    predict_pose_imu_odom.x = previous_pose.x + offset_imu_odom_x;
    predict_pose_imu_odom.y = previous_pose.y + offset_imu_odom_y;
    predict_pose_imu_odom.z = previous_pose.z + offset_imu_odom_z;
    predict_pose_imu_odom.roll = previous_pose.roll + offset_imu_odom_roll;
    predict_pose_imu_odom.pitch = previous_pose.pitch + offset_imu_odom_pitch;
    predict_pose_imu_odom.yaw = previous_pose.yaw + offset_imu_odom_yaw;

    previous_time = current_time;
}

void core::odom_calc(ros::Time current_time){
    static ros::Time previous_time = current_time;
    double diff_time = (current_time - previous_time).toSec();

    double diff_odom_roll = odom.twist.twist.angular.x * diff_time;
    double diff_odom_pitch = odom.twist.twist.angular.y * diff_time;
    double diff_odom_yaw = odom.twist.twist.angular.z * diff_time;

    current_pose_odom.roll += diff_odom_roll;
    current_pose_odom.pitch += diff_odom_pitch;
    current_pose_odom.yaw += diff_odom_yaw;

    double diff_distance = odom.twist.twist.linear.x * diff_time;
    offset_odom_x += diff_distance * cos(-current_pose_odom.pitch) * cos(current_pose_odom.yaw);
    offset_odom_y += diff_distance * cos(-current_pose_odom.pitch) * sin(current_pose_odom.yaw);
    offset_odom_z += diff_distance * sin(-current_pose_odom.pitch);

    offset_odom_roll += diff_odom_roll;
    offset_odom_pitch += diff_odom_pitch;
    offset_odom_yaw += diff_odom_yaw;

    predict_pose_odom.x = previous_pose.x + offset_odom_x;
    predict_pose_odom.y = previous_pose.y + offset_odom_y;
    predict_pose_odom.z = previous_pose.z + offset_odom_z;
    predict_pose_odom.roll = previous_pose.roll + offset_odom_roll;
    predict_pose_odom.pitch = previous_pose.pitch + offset_odom_pitch;
    predict_pose_odom.yaw = previous_pose.yaw + offset_odom_yaw;

    previous_time = current_time;
}

void core::imu_calc(ros::Time current_time){
    static ros::Time previous_time = current_time;
    double diff_time = (current_time - previous_time).toSec();

    double diff_imu_roll = imu.angular_velocity.x * diff_time;
    double diff_imu_pitch = imu.angular_velocity.y * diff_time;
    double diff_imu_yaw = imu.angular_velocity.z * diff_time;

    current_pose_imu.roll += diff_imu_roll;
    current_pose_imu.pitch += diff_imu_pitch;
    current_pose_imu.yaw += diff_imu_yaw;

    double accX1 = imu.linear_acceleration.x;
    double accY1 = std::cos(current_pose_imu.roll) * imu.linear_acceleration.y -
        std::sin(current_pose_imu.roll) * imu.linear_acceleration.z;
    double accZ1 = std::sin(current_pose_imu.roll) * imu.linear_acceleration.y +
        std::cos(current_pose_imu.roll) * imu.linear_acceleration.z;

    double accX2 = std::sin(current_pose_imu.pitch) * accZ1 + std::cos(current_pose_imu.pitch) * accX1;
    double accY2 = accY1;
    double accZ2 = std::cos(current_pose_imu.pitch) * accZ1 - std::sin(current_pose_imu.pitch) * accX1;

    double accX = std::cos(current_pose_imu.yaw) * accX2 - std::sin(current_pose_imu.yaw) * accY2;
    double accY = std::sin(current_pose_imu.yaw) * accX2 + std::cos(current_pose_imu.yaw) * accY2;
    double accZ = accZ2;

    offset_imu_x += current_velocity_imu_x * diff_time + accX * diff_time * diff_time / 2.0;
    offset_imu_y += current_velocity_imu_y * diff_time + accY * diff_time * diff_time / 2.0;
    offset_imu_z += current_velocity_imu_z * diff_time + accZ * diff_time * diff_time / 2.0;

    current_velocity_imu_x += accX * diff_time;
    current_velocity_imu_y += accY * diff_time;
    current_velocity_imu_z += accZ * diff_time;

    offset_imu_roll += diff_imu_roll;
    offset_imu_pitch += diff_imu_pitch;
    offset_imu_yaw += diff_imu_yaw;

    predict_pose_imu.x = previous_pose.x + offset_imu_x;
    predict_pose_imu.y = previous_pose.y + offset_imu_y;
    predict_pose_imu.z = previous_pose.z + offset_imu_z;
    predict_pose_imu.roll = previous_pose.roll + offset_imu_roll;
    predict_pose_imu.pitch = previous_pose.pitch + offset_imu_pitch;
    predict_pose_imu.yaw = previous_pose.yaw + offset_imu_yaw;

    previous_time = current_time;
}

const double core::wrapToPm(double a_num, const double a_max){
  if (a_num >= a_max)
  {
    a_num -= 2.0 * a_max;
  }
  return a_num;
}

const double core::wrapToPmPi(double a_angle_rad){
  return wrapToPm(a_angle_rad, M_PI);
}

void core::odom_callback(const nav_msgs::Odometry::ConstPtr& input){
  // std::cout << __func__ << std::endl;

  odom = *input;
  odom_calc(input->header.stamp);
}

void core::imuUpsideDown(const sensor_msgs::Imu::Ptr input){
 double input_roll, input_pitch, input_yaw;

 tf::Quaternion input_orientation;
 tf::quaternionMsgToTF(input->orientation, input_orientation);
 tf::Matrix3x3(input_orientation).getRPY(input_roll, input_pitch, input_yaw);

 input->angular_velocity.x *= -1;
 input->angular_velocity.y *= -1;
 input->angular_velocity.z *= -1;

 input->linear_acceleration.x *= -1;
 input->linear_acceleration.y *= -1;
 input->linear_acceleration.z *= -1;

 input_roll *= -1;
 input_pitch *= -1;
 input_yaw *= -1;

 input->orientation = tf::createQuaternionMsgFromRollPitchYaw(input_roll, input_pitch, input_yaw);
}

void core::imu_callback(const sensor_msgs::Imu::Ptr& input){
 // std::cout << __func__ << std::endl;

 if (_imu_upside_down)
   imuUpsideDown(input);

 const ros::Time current_time = input->header.stamp;
 static ros::Time previous_time = current_time;
 const double diff_time = (current_time - previous_time).toSec();

 double imu_roll, imu_pitch, imu_yaw;
 tf::Quaternion imu_orientation;
 tf::quaternionMsgToTF(input->orientation, imu_orientation);
 tf::Matrix3x3(imu_orientation).getRPY(imu_roll, imu_pitch, imu_yaw);

 imu_roll = wrapToPmPi(imu_roll);
 imu_pitch = wrapToPmPi(imu_pitch);
 imu_yaw = wrapToPmPi(imu_yaw);

 static double previous_imu_roll = imu_roll, previous_imu_pitch = imu_pitch, previous_imu_yaw = imu_yaw;
 const double diff_imu_roll = imu_roll - previous_imu_roll;

 const double diff_imu_pitch = imu_pitch - previous_imu_pitch;

 double diff_imu_yaw;
 if (fabs(imu_yaw - previous_imu_yaw) > M_PI)
 {
   if (imu_yaw > 0)
     diff_imu_yaw = (imu_yaw - previous_imu_yaw) - M_PI * 2;
   else
     diff_imu_yaw = -M_PI * 2 - (imu_yaw - previous_imu_yaw);
 }
 else
   diff_imu_yaw = imu_yaw - previous_imu_yaw;

 imu.header = input->header;
 imu.linear_acceleration.x = input->linear_acceleration.x;
 // imu.linear_acceleration.y = input->linear_acceleration.y;
 // imu.linear_acceleration.z = input->linear_acceleration.z;
 imu.linear_acceleration.y = 0;
 imu.linear_acceleration.z = 0;

 if (diff_time != 0)
 {
   imu.angular_velocity.x = diff_imu_roll / diff_time;
   imu.angular_velocity.y = diff_imu_pitch / diff_time;
   imu.angular_velocity.z = diff_imu_yaw / diff_time;
 }
 else
 {
   imu.angular_velocity.x = 0;
   imu.angular_velocity.y = 0;
   imu.angular_velocity.z = 0;
 }

 imu_calc(input->header.stamp);

 previous_time = current_time;
 previous_imu_roll = imu_roll;
 previous_imu_pitch = imu_pitch;
 previous_imu_yaw = imu_yaw;
}

void core::points_callback(const sensor_msgs::PointCloud2::ConstPtr& input){
 if (map_loaded == 1 && init_pos_set == 1)
 {
   matching_start = std::chrono::system_clock::now();

   static tf::TransformBroadcaster br;
   tf::Transform transform;
   tf::Quaternion predict_q, ndt_q, current_q, localizer_q;

   pcl::PointXYZ p;
   pcl::PointCloud<pcl::PointXYZ> filtered_scan;

   current_scan_time = input->header.stamp;

   pcl::fromROSMsg(*input, filtered_scan);
   pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_scan_ptr(new pcl::PointCloud<pcl::PointXYZ>(filtered_scan));
   int scan_points_num = filtered_scan_ptr->size();

   Eigen::Matrix4f t(Eigen::Matrix4f::Identity());   // base_link
   Eigen::Matrix4f t2(Eigen::Matrix4f::Identity());  // localizer

   std::chrono::time_point<std::chrono::system_clock> align_start, align_end, getFitnessScore_start,
       getFitnessScore_end;
   static double align_time, getFitnessScore_time = 0.0;

   pthread_mutex_lock(&mutex);

   ndt.setInputSource(filtered_scan_ptr);


   // Guess the initial gross estimation of the transformation
   predict_pose.x = previous_pose.x + offset_x;
   predict_pose.y = previous_pose.y + offset_y;
   predict_pose.z = previous_pose.z + offset_z;
   predict_pose.roll = previous_pose.roll;
   predict_pose.pitch = previous_pose.pitch;
   predict_pose.yaw = previous_pose.yaw + offset_yaw;

   if (_use_imu == true && _use_odom == true)
     imu_odom_calc(current_scan_time);
   if (_use_imu == true && _use_odom == false)
     imu_calc(current_scan_time);
   if (_use_imu == false && _use_odom == true)
     odom_calc(current_scan_time);

   pose predict_pose_for_ndt;
   if (_use_imu == true && _use_odom == true)
     predict_pose_for_ndt = predict_pose_imu_odom;
   else if (_use_imu == true && _use_odom == false)
     predict_pose_for_ndt = predict_pose_imu;
   else if (_use_imu == false && _use_odom == true)
     predict_pose_for_ndt = predict_pose_odom;
   else
     predict_pose_for_ndt = predict_pose;

   Eigen::Translation3f init_translation(predict_pose_for_ndt.x, predict_pose_for_ndt.y, predict_pose_for_ndt.z);
   Eigen::AngleAxisf init_rotation_x(predict_pose_for_ndt.roll, Eigen::Vector3f::UnitX());
   Eigen::AngleAxisf init_rotation_y(predict_pose_for_ndt.pitch, Eigen::Vector3f::UnitY());
   Eigen::AngleAxisf init_rotation_z(predict_pose_for_ndt.yaw, Eigen::Vector3f::UnitZ());
   Eigen::Matrix4f init_guess = (init_translation * init_rotation_z * init_rotation_y * init_rotation_x) * tf_btol;

   pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);

   align_start = std::chrono::system_clock::now();
   ndt.align(*output_cloud, init_guess);
   align_end = std::chrono::system_clock::now();
   has_converged = ndt.hasConverged();
   t = ndt.getFinalTransformation();
   iteration = ndt.getFinalNumIteration();
   getFitnessScore_start = std::chrono::system_clock::now();
   fitness_score = ndt.getFitnessScore();
   getFitnessScore_end = std::chrono::system_clock::now();
   trans_probability = ndt.getTransformationProbability();


   align_time = std::chrono::duration_cast<std::chrono::microseconds>(align_end - align_start).count() / 1000.0;

   t2 = t * tf_btol.inverse();

   getFitnessScore_time =
       std::chrono::duration_cast<std::chrono::microseconds>(getFitnessScore_end - getFitnessScore_start).count() /
       1000.0;

   pthread_mutex_unlock(&mutex);

   tf::Matrix3x3 mat_l;  // localizer
   mat_l.setValue(static_cast<double>(t(0, 0)), static_cast<double>(t(0, 1)), static_cast<double>(t(0, 2)),
                  static_cast<double>(t(1, 0)), static_cast<double>(t(1, 1)), static_cast<double>(t(1, 2)),
                  static_cast<double>(t(2, 0)), static_cast<double>(t(2, 1)), static_cast<double>(t(2, 2)));

   // Update localizer_pose
   localizer_pose.x = t(0, 3);
   localizer_pose.y = t(1, 3);
   localizer_pose.z = t(2, 3);
   mat_l.getRPY(localizer_pose.roll, localizer_pose.pitch, localizer_pose.yaw, 1);

   tf::Matrix3x3 mat_b;  // base_link
   mat_b.setValue(static_cast<double>(t2(0, 0)), static_cast<double>(t2(0, 1)), static_cast<double>(t2(0, 2)),
                  static_cast<double>(t2(1, 0)), static_cast<double>(t2(1, 1)), static_cast<double>(t2(1, 2)),
                  static_cast<double>(t2(2, 0)), static_cast<double>(t2(2, 1)), static_cast<double>(t2(2, 2)));

   // Update ndt_pose
   ndt_pose.x = t2(0, 3);
   ndt_pose.y = t2(1, 3);
   ndt_pose.z = t2(2, 3);
   mat_b.getRPY(ndt_pose.roll, ndt_pose.pitch, ndt_pose.yaw, 1);

   // Calculate the difference between ndt_pose and predict_pose
   predict_pose_error = sqrt((ndt_pose.x - predict_pose_for_ndt.x) * (ndt_pose.x - predict_pose_for_ndt.x) +
                             (ndt_pose.y - predict_pose_for_ndt.y) * (ndt_pose.y - predict_pose_for_ndt.y) +
                             (ndt_pose.z - predict_pose_for_ndt.z) * (ndt_pose.z - predict_pose_for_ndt.z));

   if (predict_pose_error <= PREDICT_POSE_THRESHOLD)
   {
     use_predict_pose = 0;
   }
   else
   {
     use_predict_pose = 1;
   }
   use_predict_pose = 0;

   if (use_predict_pose == 0)
   {
     current_pose.x = ndt_pose.x;
     current_pose.y = ndt_pose.y;
     current_pose.z = ndt_pose.z;
     current_pose.roll = ndt_pose.roll;
     current_pose.pitch = ndt_pose.pitch;
     current_pose.yaw = ndt_pose.yaw;
   }
   else
   {
     current_pose.x = predict_pose_for_ndt.x;
     current_pose.y = predict_pose_for_ndt.y;
     current_pose.z = predict_pose_for_ndt.z;
     current_pose.roll = predict_pose_for_ndt.roll;
     current_pose.pitch = predict_pose_for_ndt.pitch;
     current_pose.yaw = predict_pose_for_ndt.yaw;
   }

   // Compute the velocity and acceleration
   scan_duration = current_scan_time - previous_scan_time;
   double secs = scan_duration.toSec();
   diff_x = current_pose.x - previous_pose.x;
   diff_y = current_pose.y - previous_pose.y;
   diff_z = current_pose.z - previous_pose.z;
   diff_yaw = current_pose.yaw - previous_pose.yaw;
   diff = sqrt(diff_x * diff_x + diff_y * diff_y + diff_z * diff_z);

   current_velocity = diff / secs;
   current_velocity_x = diff_x / secs;
   current_velocity_y = diff_y / secs;
   current_velocity_z = diff_z / secs;
   angular_velocity = diff_yaw / secs;

   current_pose_imu.x = current_pose.x;
   current_pose_imu.y = current_pose.y;
   current_pose_imu.z = current_pose.z;
   current_pose_imu.roll = current_pose.roll;
   current_pose_imu.pitch = current_pose.pitch;
   current_pose_imu.yaw = current_pose.yaw;

   current_velocity_imu_x = current_velocity_x;
   current_velocity_imu_y = current_velocity_y;
   current_velocity_imu_z = current_velocity_z;

   current_pose_odom.x = current_pose.x;
   current_pose_odom.y = current_pose.y;
   current_pose_odom.z = current_pose.z;
   current_pose_odom.roll = current_pose.roll;
   current_pose_odom.pitch = current_pose.pitch;
   current_pose_odom.yaw = current_pose.yaw;

   current_pose_imu_odom.x = current_pose.x;
   current_pose_imu_odom.y = current_pose.y;
   current_pose_imu_odom.z = current_pose.z;
   current_pose_imu_odom.roll = current_pose.roll;
   current_pose_imu_odom.pitch = current_pose.pitch;
   current_pose_imu_odom.yaw = current_pose.yaw;

   current_velocity_smooth = (current_velocity + previous_velocity + previous_previous_velocity) / 3.0;
   if (current_velocity_smooth < 0.2)
   {
     current_velocity_smooth = 0.0;
   }

   current_accel = (current_velocity - previous_velocity) / secs;
   current_accel_x = (current_velocity_x - previous_velocity_x) / secs;
   current_accel_y = (current_velocity_y - previous_velocity_y) / secs;
   current_accel_z = (current_velocity_z - previous_velocity_z) / secs;

   estimated_vel_mps.data = current_velocity;
   estimated_vel_kmph.data = current_velocity * 3.6;

   estimated_vel_mps_pub.publish(estimated_vel_mps);
   estimated_vel_kmph_pub.publish(estimated_vel_kmph);

   // Set values for publishing pose
   predict_q.setRPY(predict_pose.roll, predict_pose.pitch, predict_pose.yaw);

   predict_pose_msg.header.frame_id = "/map";
   predict_pose_msg.header.stamp = current_scan_time;
   predict_pose_msg.pose.position.x = predict_pose.x;
   predict_pose_msg.pose.position.y = predict_pose.y;
   predict_pose_msg.pose.position.z = predict_pose.z;
   predict_pose_msg.pose.orientation.x = predict_q.x();
   predict_pose_msg.pose.orientation.y = predict_q.y();
   predict_pose_msg.pose.orientation.z = predict_q.z();
   predict_pose_msg.pose.orientation.w = predict_q.w();


   tf::Quaternion predict_q_imu;
   predict_q_imu.setRPY(predict_pose_imu.roll, predict_pose_imu.pitch, predict_pose_imu.yaw);
   predict_pose_imu_msg.header.frame_id = "map";
   predict_pose_imu_msg.header.stamp = input->header.stamp;
   predict_pose_imu_msg.pose.position.x = predict_pose_imu.x;
   predict_pose_imu_msg.pose.position.y = predict_pose_imu.y;
   predict_pose_imu_msg.pose.position.z = predict_pose_imu.z;
   predict_pose_imu_msg.pose.orientation.x = predict_q_imu.x();
   predict_pose_imu_msg.pose.orientation.y = predict_q_imu.y();
   predict_pose_imu_msg.pose.orientation.z = predict_q_imu.z();
   predict_pose_imu_msg.pose.orientation.w = predict_q_imu.w();
   predict_pose_imu_pub.publish(predict_pose_imu_msg);

   tf::Quaternion predict_q_odom;
   predict_q_odom.setRPY(predict_pose_odom.roll, predict_pose_odom.pitch, predict_pose_odom.yaw);
   predict_pose_odom_msg.header.frame_id = "map";
   predict_pose_odom_msg.header.stamp = input->header.stamp;
   predict_pose_odom_msg.pose.position.x = predict_pose_odom.x;
   predict_pose_odom_msg.pose.position.y = predict_pose_odom.y;
   predict_pose_odom_msg.pose.position.z = predict_pose_odom.z;
   predict_pose_odom_msg.pose.orientation.x = predict_q_odom.x();
   predict_pose_odom_msg.pose.orientation.y = predict_q_odom.y();
   predict_pose_odom_msg.pose.orientation.z = predict_q_odom.z();
   predict_pose_odom_msg.pose.orientation.w = predict_q_odom.w();
   predict_pose_odom_pub.publish(predict_pose_odom_msg);

   tf::Quaternion predict_q_imu_odom;
   predict_q_imu_odom.setRPY(predict_pose_imu_odom.roll, predict_pose_imu_odom.pitch, predict_pose_imu_odom.yaw);
   predict_pose_imu_odom_msg.header.frame_id = "map";
   predict_pose_imu_odom_msg.header.stamp = input->header.stamp;
   predict_pose_imu_odom_msg.pose.position.x = predict_pose_imu_odom.x;
   predict_pose_imu_odom_msg.pose.position.y = predict_pose_imu_odom.y;
   predict_pose_imu_odom_msg.pose.position.z = predict_pose_imu_odom.z;
   predict_pose_imu_odom_msg.pose.orientation.x = predict_q_imu_odom.x();
   predict_pose_imu_odom_msg.pose.orientation.y = predict_q_imu_odom.y();
   predict_pose_imu_odom_msg.pose.orientation.z = predict_q_imu_odom.z();
   predict_pose_imu_odom_msg.pose.orientation.w = predict_q_imu_odom.w();
   predict_pose_imu_odom_pub.publish(predict_pose_imu_odom_msg);

   ndt_q.setRPY(ndt_pose.roll, ndt_pose.pitch, ndt_pose.yaw);

   ndt_pose_msg.header.frame_id = "/map";
   ndt_pose_msg.header.stamp = current_scan_time;
   ndt_pose_msg.pose.pose.position.x = ndt_pose.x;
   ndt_pose_msg.pose.pose.position.y = ndt_pose.y;
   ndt_pose_msg.pose.pose.position.z = ndt_pose.z;
   ndt_pose_msg.pose.pose.orientation.x = ndt_q.x();
   ndt_pose_msg.pose.pose.orientation.y = ndt_q.y();
   ndt_pose_msg.pose.pose.orientation.z = ndt_q.z();
   ndt_pose_msg.pose.pose.orientation.w = ndt_q.w();

   current_q.setRPY(current_pose.roll, current_pose.pitch, current_pose.yaw);
   // current_pose is published by vel_pose_mux
   /*
   current_pose_msg.header.frame_id = "/map";
   current_pose_msg.header.stamp = current_scan_time;
   current_pose_msg.pose.position.x = current_pose.x;
   current_pose_msg.pose.position.y = current_pose.y;
   current_pose_msg.pose.position.z = current_pose.z;
   current_pose_msg.pose.orientation.x = current_q.x();
   current_pose_msg.pose.orientation.y = current_q.y();
   current_pose_msg.pose.orientation.z = current_q.z();
   current_pose_msg.pose.orientation.w = current_q.w();
   */

   localizer_q.setRPY(localizer_pose.roll, localizer_pose.pitch, localizer_pose.yaw);

   localizer_pose_msg.header.frame_id = "/map";
   localizer_pose_msg.header.stamp = current_scan_time;
   localizer_pose_msg.pose.position.x = localizer_pose.x;
   localizer_pose_msg.pose.position.y = localizer_pose.y;
   localizer_pose_msg.pose.position.z = localizer_pose.z;
   localizer_pose_msg.pose.orientation.x = localizer_q.x();
   localizer_pose_msg.pose.orientation.y = localizer_q.y();
   localizer_pose_msg.pose.orientation.z = localizer_q.z();
   localizer_pose_msg.pose.orientation.w = localizer_q.w();


   predict_pose_pub.publish(predict_pose_msg);
   ndt_pose_pub.publish(ndt_pose_msg);
   // current_pose is published by vel_pose_mux
   //    current_pose_pub.publish(current_pose_msg);
   localizer_pose_pub.publish(localizer_pose_msg);

   // Send TF "/base_link" to "/map"
   transform.setOrigin(tf::Vector3(current_pose.x, current_pose.y, current_pose.z));
   transform.setRotation(current_q);

   odomPub(current_scan_time, current_pose.x, current_pose.y, current_pose.yaw, current_velocity_x, current_velocity_y, angular_velocity);

   br.sendTransform(tf::StampedTransform(transform, current_scan_time, "/map", "/base_link"));

   matching_end = std::chrono::system_clock::now();
   exe_time = std::chrono::duration_cast<std::chrono::microseconds>(matching_end - matching_start).count() / 1000.0;
   time_ndt_matching.data = exe_time;
   time_ndt_matching_pub.publish(time_ndt_matching);

   // Set values for /estimate_twist
   estimate_twist_msg.header.stamp = current_scan_time;
   estimate_twist_msg.header.frame_id = "/base_link";
   estimate_twist_msg.twist.linear.x = current_velocity;
   estimate_twist_msg.twist.linear.y = 0.0;
   estimate_twist_msg.twist.linear.z = 0.0;
   estimate_twist_msg.twist.angular.x = 0.0;
   estimate_twist_msg.twist.angular.y = 0.0;
   estimate_twist_msg.twist.angular.z = angular_velocity;

   estimate_twist_pub.publish(estimate_twist_msg);

   geometry_msgs::Vector3Stamped estimate_vel_msg;
   estimate_vel_msg.header.stamp = current_scan_time;
   estimate_vel_msg.vector.x = current_velocity;
   estimated_vel_pub.publish(estimate_vel_msg);

   // Set values for /ndt_stat
 ndt_stat_msg.header.stamp = current_scan_time;
 ndt_stat_msg.exe_time = time_ndt_matching.data;
 ndt_stat_msg.iteration = iteration;
 ndt_stat_msg.score = fitness_score;
 ndt_stat_msg.velocity = current_velocity;
 ndt_stat_msg.acceleration = current_accel;
 ndt_stat_msg.use_predict_pose = 0;

 ndt_stat_pub.publish(ndt_stat_msg);
   /* Compute NDT_Reliability */
   ndt_reliability.data = Wa * (exe_time / 100.0) * 100.0 + Wb * (iteration / 10.0) * 100.0 +
       Wc * ((2.0 - trans_probability) / 2.0) * 100.0;
   ndt_reliability_pub.publish(ndt_reliability);

   // Write log
   if (!ofs)
   {
     std::cerr << "Could not open " << filename << "." << std::endl;
     exit(1);
   }
   static ros::Time start_time = input->header.stamp;

   /*ofs << input->header.seq << "," << scan_points_num << "," << step_size << "," << trans_eps << "," << std::fixed
       << std::setprecision(5) << current_pose.x << "," << std::fixed << std::setprecision(5) << current_pose.y << ","
       << std::fixed << std::setprecision(5) << current_pose.z << "," << current_pose.roll << "," << current_pose.pitch
       << "," << current_pose.yaw << "," << predict_pose.x << "," << predict_pose.y << "," << predict_pose.z << ","
       << predict_pose.roll << "," << predict_pose.pitch << "," << predict_pose.yaw << ","
       << current_pose.x - predict_pose.x << "," << current_pose.y - predict_pose.y << ","
       << current_pose.z - predict_pose.z << "," << current_pose.roll - predict_pose.roll << ","
       << current_pose.pitch - predict_pose.pitch << "," << current_pose.yaw - predict_pose.yaw << ","
       << predict_pose_error << "," << iteration << "," << fitness_score << "," << trans_probability << ","
       << ndt_reliability.data << "," << current_velocity << "," << current_velocity_smooth << "," << current_accel
       << "," << angular_velocity << "," << time_ndt_matching.data << "," << align_time << "," << getFitnessScore_time
       << std::endl;*/

   std::cout << "-----------------------------------------------------------------" << std::endl;
   std::cout << "Sequence: " << input->header.seq << std::endl;
   std::cout << "Timestamp: " << input->header.stamp << std::endl;
   std::cout << "Frame ID: " << input->header.frame_id << std::endl;
   //		std::cout << "Number of Scan Points: " << scan_ptr->size() << " points." << std::endl;
   std::cout << "Number of Filtered Scan Points: " << scan_points_num << " points." << std::endl;
   std::cout << "NDT has converged: " << has_converged << std::endl;
   std::cout << "Fitness Score: " << fitness_score << std::endl;
   std::cout << "Transformation Probability: " << trans_probability << std::endl;
   std::cout << "Execution Time: " << exe_time << " ms." << std::endl;
   std::cout << "Number of Iterations: " << iteration << std::endl;
   std::cout << "NDT Reliability: " << ndt_reliability.data << std::endl;
   std::cout << "(x,y,z,roll,pitch,yaw): " << std::endl;
   std::cout << "(" << current_pose.x << ", " << current_pose.y << ", " << current_pose.z << ", " << current_pose.roll
             << ", " << current_pose.pitch << ", " << current_pose.yaw << ")" << std::endl;
   std::cout << "Transformation Matrix: " << std::endl;
   std::cout << t << std::endl;
   std::cout << "Align time: " << align_time << std::endl;
   std::cout << "Get fitness score time: " << getFitnessScore_time << std::endl;
   std::cout << "-----------------------------------------------------------------" << std::endl;

   // Update offset
   if (_offset == "linear")
   {
     offset_x = diff_x;
     offset_y = diff_y;
     offset_z = diff_z;
     offset_yaw = diff_yaw;
   }
   else if (_offset == "quadratic")
   {
     offset_x = (current_velocity_x + current_accel_x * secs) * secs;
     offset_y = (current_velocity_y + current_accel_y * secs) * secs;
     offset_z = diff_z;
     offset_yaw = diff_yaw;
   }
   else if (_offset == "zero")
   {
     offset_x = 0.0;
     offset_y = 0.0;
     offset_z = 0.0;
     offset_yaw = 0.0;
   }

   offset_imu_x = 0.0;
   offset_imu_y = 0.0;
   offset_imu_z = 0.0;
   offset_imu_roll = 0.0;
   offset_imu_pitch = 0.0;
   offset_imu_yaw = 0.0;

   offset_odom_x = 0.0;
   offset_odom_y = 0.0;
   offset_odom_z = 0.0;
   offset_odom_roll = 0.0;
   offset_odom_pitch = 0.0;
   offset_odom_yaw = 0.0;

   offset_imu_odom_x = 0.0;
   offset_imu_odom_y = 0.0;
   offset_imu_odom_z = 0.0;
   offset_imu_odom_roll = 0.0;
   offset_imu_odom_pitch = 0.0;
   offset_imu_odom_yaw = 0.0;

   // Update previous_***
   previous_pose.x = current_pose.x;
   previous_pose.y = current_pose.y;
   previous_pose.z = current_pose.z;
   previous_pose.roll = current_pose.roll;
   previous_pose.pitch = current_pose.pitch;
   previous_pose.yaw = current_pose.yaw;

   previous_scan_time.sec = current_scan_time.sec;
   previous_scan_time.nsec = current_scan_time.nsec;

   previous_previous_velocity = previous_velocity;
   previous_velocity = current_velocity;
   previous_velocity_x = current_velocity_x;
   previous_velocity_y = current_velocity_y;
   previous_velocity_z = current_velocity_z;
   previous_accel = current_accel;

   previous_estimated_vel_kmph.data = estimated_vel_kmph.data;
 }

}

}
