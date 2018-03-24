/*
 *  Copyright (c) 2015, Nagoya University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/*
 Message counter in subscriber's queue.

 Yuki KITSUKAWA
 */

#include <iostream>
#include <sstream>
#include <fstream>
#include <string>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/PointCloud2.h>

static int enqueue = 0;
static int dequeue = 0;
std_msgs::Bool save;
ros::Publisher pub;
static void points_callback(const sensor_msgs::PointCloud2::ConstPtr& input)
{
	enqueue++;
}

static void ndt_map_callback(const sensor_msgs::PointCloud2::ConstPtr& input)
{
	dequeue++;

        save.data=true;
       	pub.publish(save);
	std::cout << "(Processed/Input): (" << dequeue << " / " << enqueue << ")" << std::endl;
     	std::cout << "saved \n"<< std::endl;
      
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "queue_counter");

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    ros::Subscriber points_sub = nh.subscribe("/velodyne/velodyne_points", 100000, points_callback);
    ros::Subscriber ndt_map_sub = nh.subscribe("ndt_map", 100000, ndt_map_callback);
    pub = nh.advertise<std_msgs::Bool>("save",100000);

    ros::spin();

    return 0;
}
