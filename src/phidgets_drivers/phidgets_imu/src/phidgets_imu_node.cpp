#include "phidgets_imu/imu_ros_i.h"

int main(int argc, char **argv)
{
  ros::init (argc, argv, "PhidgetsImu");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  phidgets::ImuRosI imu(nh, nh_private);
  ros::spin();
  return 0;
}
