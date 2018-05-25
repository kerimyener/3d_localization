#include "common.hh"
using namespace localizer;
int main(int argc, char** argv)
{
  ros::init(argc, argv, "ndt_matching");
  core NDT_core;
    NDT_core.toInitNDT();
  return 0;
}
