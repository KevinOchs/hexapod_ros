#ifndef __FOVIS_ROS_VISUALIZATION_H__
#define __FOVIS_ROS_VISUALIZATION_H__

namespace cv
{
  class Mat;
}

namespace fovis
{
  class VisualOdometry;
}

namespace fovis_ros
{

namespace visualization
{
  cv::Mat paint(const fovis::VisualOdometry* odometry);
} // end of namespace visualization

} // end of namespace fovis_ros

#endif
