#include "ndt_matching.hh"

using namespace ndt;
/** Main node entry point. */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "ndtMatching");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  NDT_MATCH ndtMatch;
  ndtMatch.setup(nh,private_nh);
  ros::spin();
  return 0;
}
