#include "odometry/odometry.hh"

using namespace odometry;

void laserOdometry::poseCallback(geometry_msgs::PoseWithCovarianceStampedConstPtr msg){
    // Store odometry input values
    if( prev_time == 0 ) {
        prev_time = msg->header.stamp.toSec();
        return;
    }
    d_time = msg->header.stamp.toSec() - prev_time;

    x= msg->pose.pose.position.x;
    y= msg->pose.pose.position.y;
    yaw= msg->pose.pose.orientation.z;

    prev_time = msg->header.stamp.toSec();


    updateOdometry();

    last_update_time = ros::Time::now();
    valid_data = true;
}

void laserOdometry::updateOdometry() {
    // Source: http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom
    // Source: https://github.com/linorobot/linorobot/wiki/3.-Odometry


    d_x = x - previous_x;
    d_y = y - previous_y;
    d_yaw = yaw - previous_yaw;

    v_x = d_x / d_time;
    v_y = d_y / d_time;
    v_yaw = d_yaw / d_time;

    previous_x = x;
    previous_y = y;
    previous_yaw = yaw;

        // approximately straight steering - turn radius +inf

}

void laserOdometry::publishOdometry(const ros::TimerEvent&) {

    if( !valid_data ) {
        return;
    }

    // Transform frame
    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odomQuat = tf::createQuaternionMsgFromYaw(yaw);

    if (publish_tf) {
        geometry_msgs::TransformStamped odomTrans;
        odomTrans.header.stamp = last_update_time;
        odomTrans.header.frame_id = "map";
        odomTrans.child_frame_id = "base_link";

        odomTrans.transform.translation.x = x;
        odomTrans.transform.translation.y = y;
        odomTrans.transform.translation.z = 0.0;
        odomTrans.transform.rotation = odomQuat;

        odomBroadcaster->sendTransform(odomTrans);
    }

    // Odometry message
    nav_msgs::Odometry odom;
    odom.header.stamp = last_update_time;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odomQuat;

    //set the velocity
    odom.twist.twist.linear.x = v_x;
    odom.twist.twist.linear.y = v_y;
    odom.twist.twist.angular.z = v_yaw;

    odomPub.publish(odom);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "mia_odometry_node");
    ros::NodeHandle n;
    ros::NodeHandle nh("~");

    bool publish_tf;
    nh.param<bool>("publish_tf", publish_tf, true);
    /*nh.param<double>("wheelbase", wheelbase, 1.915);
    nh.param<double>("wheel_radius", wheel_radius, 0.265);*/

    laserOdometry miaOdometry(publish_tf);
    
    ros::Subscriber sub = n.subscribe("ndt_pose", 1000,
            &laserOdometry::poseCallback, &miaOdometry);

    miaOdometry.odomPub = n.advertise<nav_msgs::Odometry>("laser_odometry", 1000);
    miaOdometry.odomBroadcaster = boost::make_shared<tf::TransformBroadcaster>();
    ros::Timer odomTimer = n.createTimer(ros::Duration(1.0/10.0/*10 Hz*/),
            &laserOdometry::publishOdometry, &miaOdometry);
    ROS_INFO("Mia odometry running");
    ros::spin();

    return 0;
}
