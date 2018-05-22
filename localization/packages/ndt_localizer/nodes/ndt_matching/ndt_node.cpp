#include "localization.h"
using namespace localizer;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ndt_matching");
  core ndt_core;
  ndt_core.toInitNdt();

  return 0;

}
