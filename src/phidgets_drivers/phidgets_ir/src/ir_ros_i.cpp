#include "phidgets_ir/ir_ros_i.h"

namespace phidgets {

IRRosI::IRRosI(ros::NodeHandle nh, ros::NodeHandle nh_private):
  IR(),
  nh_(nh), 
  nh_private_(nh_private)
{
  ROS_INFO ("Starting Phidgets IR");

  initDevice();
}

void IRRosI::initDevice()
{
	ROS_INFO("Opening device");
	open(-1);

	ROS_INFO("Waiting for IR to be attached...");
	int result = waitForAttachment(10000);
	if(result)
	{
	  const char *err;
		CPhidget_getErrorDescription(result, &err);
		ROS_FATAL("Problem waiting for IR attachment: %s", err);
	}
}

void IRRosI::codeHandler(unsigned char *data, int dataLength, int bitCount, int repeat)
{
  // do nothing - just refer to base class callbalck, which prints the values
	IR::codeHandler(data, dataLength, bitCount, repeat);
}

} // namespace phidgets

