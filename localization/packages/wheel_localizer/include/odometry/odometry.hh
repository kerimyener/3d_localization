#ifndef _ODOMETRY_HH_
#define _ODOMETRY_HH_

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_broadcaster.h>
#include <math.h>


namespace odometry {
    class laserOdometry {
        public:
        laserOdometry(bool publish_tf)
              : publish_tf(publish_tf),
                x( 0 ),
                y( 0 ),
                yaw( 0 ),
                prev_time( 0 ),
                previous_x( 0 ),
                previous_y( 0 ),
                previous_yaw( 0 ),
                valid_data( false ) {}

            void poseCallback(geometry_msgs::PoseWithCovarianceStampedConstPtr msg);
            void publishOdometry(const ros::TimerEvent&);

            ros::Publisher odomPub;
            boost::shared_ptr<tf::TransformBroadcaster> odomBroadcaster;

        protected:
            bool publish_tf;

            double x;
            double y;
            double yaw;

            double d_x;
            double d_y;
            double d_yaw;
            double d_time;

            double v_x;
            double v_y;
            double v_yaw;

            double prev_time;

            double previous_x;
            double previous_y;
            double previous_yaw;


            bool valid_data;
            ros::Time last_update_time;

            void updateOdometry();
    };
}

#endif // _ODOMETRY_HH_
