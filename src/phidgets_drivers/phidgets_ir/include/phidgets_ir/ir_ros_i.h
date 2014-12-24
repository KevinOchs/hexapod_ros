#ifndef PHIDGETS_IR_IR_ROS_I_H
#define PHIDGETS_IR_IR_ROS_I_H

#include <ros/ros.h>
#include <phidgets_api/ir.h>

namespace phidgets {

class IRRosI : public IR
{

  public:

    IRRosI(ros::NodeHandle nh, ros::NodeHandle nh_private);

  private:

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    void initDevice();
    void codeHandler(unsigned char *data, int dataLength, int bitCount, int repeat);
};

} //namespace phidgets

#endif // PHIDGETS_IR_IR_ROS_I_H
