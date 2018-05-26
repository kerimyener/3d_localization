#include "localiser_core.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "nmea2tfpose");
  localiser::core ndt_core;
  ndt_core.run();

  return 0;
}
