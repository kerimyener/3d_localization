#include "ndt_mapping.hh"

using namespace ndt;
/** Main node entry point. */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "ndtMapping");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  NDT_MAPP ndtMapp;
  ndtMapp.setup(nh,private_nh);
  ros::spin();

}
