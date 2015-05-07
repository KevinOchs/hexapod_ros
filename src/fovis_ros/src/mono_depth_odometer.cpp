#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <image_geometry/stereo_camera_model.h>
#include <cv_bridge/cv_bridge.h>

#include <fovis_ros/FovisInfo.h>

#include <libfovis/depth_image.hpp>

#include "mono_depth_processor.hpp"
#include "odometer_base.hpp"
#include "visualization.hpp"

namespace fovis_ros
{

class MonoDepthOdometer : public MonoDepthProcessor, OdometerBase
{

private:

  fovis::DepthImage* depth_image_;

public:

  MonoDepthOdometer(const std::string& transport) : 
    MonoDepthProcessor(transport),
    depth_image_(NULL)
  {
  }

  ~MonoDepthOdometer()
  {
    if (depth_image_) delete depth_image_;
  }

protected:

  fovis::DepthImage* createDepthSource(
      const sensor_msgs::CameraInfoConstPtr& image_info_msg,
      const sensor_msgs::CameraInfoConstPtr& depth_info_msg) const
  {
    // read calibration info from camera info message
    image_geometry::PinholeCameraModel model;
    model.fromCameraInfo(*image_info_msg);
    
    // initialize left camera parameters
    fovis::CameraIntrinsicsParameters parameters;
    rosToFovis(model, parameters);

    return new fovis::DepthImage(parameters, 
        depth_info_msg->width, depth_info_msg->height);
  }

  void imageCallback(
      const sensor_msgs::ImageConstPtr& image_msg,
      const sensor_msgs::ImageConstPtr& depth_msg,
      const sensor_msgs::CameraInfoConstPtr& image_info_msg,
      const sensor_msgs::CameraInfoConstPtr& depth_info_msg)
  {
    if (!depth_image_)
    {
      depth_image_ = createDepthSource(image_info_msg, depth_info_msg);
      setDepthSource(depth_image_);
    }

    if (depth_msg->encoding != sensor_msgs::image_encodings::TYPE_32FC1)
    {
      ROS_ERROR("Depth image must be in 32bit floating point format!");
      return;
    }
    ROS_ASSERT(depth_msg->step == depth_msg->width * sizeof(float));
    const float* depth_data = reinterpret_cast<const float*>(depth_msg->data.data());

    // pass data to depth source
    depth_image_->setDepthImage(depth_data);

    // call base implementation
    process(image_msg, image_info_msg);
  }
};

} // end of namespace


int main(int argc, char **argv)
{
  ros::init(argc, argv, "mono_depth_odometer");
  std::string transport = argc > 1 ? argv[1] : "raw";
  fovis_ros::MonoDepthOdometer odometer(transport);
  ros::spin();
  return 0;
}

