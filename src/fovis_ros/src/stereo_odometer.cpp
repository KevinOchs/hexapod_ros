#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <image_geometry/stereo_camera_model.h>
#include <cv_bridge/cv_bridge.h>

#include <fovis_ros/FovisInfo.h>

#include <libfovis/stereo_depth.hpp>
#include <libfovis/stereo_calibration.hpp>

#include "stereo_processor.hpp"
#include "odometer_base.hpp"
#include "visualization.hpp"

namespace fovis_ros
{

class StereoOdometer : public StereoProcessor, OdometerBase
{

private:

  fovis::StereoDepth* stereo_depth_;

public:

  StereoOdometer(const std::string& transport) :
    StereoProcessor(transport),
    stereo_depth_(NULL)
  {
  }

  ~StereoOdometer()
  {
    if (stereo_depth_) delete stereo_depth_;
  }

protected:

  fovis::StereoDepth* createStereoDepth(
      const sensor_msgs::CameraInfoConstPtr& l_info_msg,
      const sensor_msgs::CameraInfoConstPtr& r_info_msg) const
  {
    // read calibration info from camera info message
    // to fill remaining parameters
    image_geometry::StereoCameraModel model;
    model.fromCameraInfo(*l_info_msg, *r_info_msg);

    // initialize left camera parameters
    fovis::CameraIntrinsicsParameters left_parameters;
    rosToFovis(model.left(), left_parameters);
    left_parameters.height = l_info_msg->height;
    left_parameters.width = l_info_msg->width;
    // initialize right camera parameters
    fovis::CameraIntrinsicsParameters right_parameters;
    rosToFovis(model.right(), right_parameters);
    right_parameters.height = r_info_msg->height;
    right_parameters.width = r_info_msg->width;

    // as we use rectified images, rotation is identity
    // and translation is baseline only
    fovis::StereoCalibrationParameters stereo_parameters;
    stereo_parameters.left_parameters = left_parameters;
    stereo_parameters.right_parameters = right_parameters;
    stereo_parameters.right_to_left_rotation[0] = 1.0;
    stereo_parameters.right_to_left_rotation[1] = 0.0;
    stereo_parameters.right_to_left_rotation[2] = 0.0;
    stereo_parameters.right_to_left_rotation[3] = 0.0;
    stereo_parameters.right_to_left_translation[0] = -model.baseline();
    stereo_parameters.right_to_left_translation[1] = 0.0;
    stereo_parameters.right_to_left_translation[2] = 0.0;

    fovis::StereoCalibration* stereo_calibration =
      new fovis::StereoCalibration(stereo_parameters);

    return new fovis::StereoDepth(stereo_calibration, getOptions());
  }

  void imageCallback(
      const sensor_msgs::ImageConstPtr& l_image_msg,
      const sensor_msgs::ImageConstPtr& r_image_msg,
      const sensor_msgs::CameraInfoConstPtr& l_info_msg,
      const sensor_msgs::CameraInfoConstPtr& r_info_msg)
  {
    if (!stereo_depth_)
    {
      stereo_depth_ = createStereoDepth(l_info_msg, r_info_msg);
      setDepthSource(stereo_depth_);
    }
    // convert image if necessary
    uint8_t *r_image_data;
    int r_step;
    cv_bridge::CvImageConstPtr r_cv_ptr;
    r_cv_ptr = cv_bridge::toCvShare(r_image_msg, sensor_msgs::image_encodings::MONO8);
    r_image_data = r_cv_ptr->image.data;
    r_step = r_cv_ptr->image.step[0];

    ROS_ASSERT(r_step == static_cast<int>(r_image_msg->width));
    ROS_ASSERT(l_image_msg->width == r_image_msg->width);
    ROS_ASSERT(l_image_msg->height == r_image_msg->height);

    // pass image to depth source
    stereo_depth_->setRightImage(r_image_data);

    // call base implementation
    process(l_image_msg, l_info_msg);
  }
};

} // end of namespace


int main(int argc, char **argv)
{
  ros::init(argc, argv, "stereo_odometer");
  if (ros::names::remap("stereo") == "stereo") {
    ROS_WARN("'stereo' has not been remapped! Example command-line usage:\n"
             "\t$ rosrun fovis_ros stereo_odometer stereo:=narrow_stereo image:=image_rect");
  }
  if (ros::names::remap("image").find("rect") == std::string::npos) {
    ROS_WARN("stereo_odometer needs rectified input images. The used image "
             "topic is '%s'. Are you sure the images are rectified?",
             ros::names::remap("image").c_str());
  }

  std::string transport = argc > 1 ? argv[1] : "raw";
  fovis_ros::StereoOdometer odometer(transport);

  ros::spin();
  return 0;
}

