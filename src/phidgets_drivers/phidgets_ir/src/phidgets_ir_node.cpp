#include "phidgets_ir/ir_ros_i.h"

int main(int argc, char **argv)
{
  ros::init (argc, argv, "PhidgetsIR");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  phidgets::IRRosI ir(nh, nh_private);
  ros::spin();
  return 0;
}
