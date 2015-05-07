#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <image_geometry/pinhole_camera_model.h>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

#include <fovis_ros/FovisInfo.h>

#include <libfovis/visual_odometry.hpp>
#include <libfovis/stereo_depth.hpp>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include "visualization.hpp"

namespace fovis_ros
{

/**
 * Base class for fovis odometers.
 */
class OdometerBase
{

protected:

  OdometerBase() : 
    visual_odometer_(NULL),
    rectification_(NULL),
    depth_source_(NULL),
    visual_odometer_options_(fovis::VisualOdometry::getDefaultOptions()),
    nh_local_("~"),
    it_(nh_local_)
  {
    loadParams();
    odom_pub_ = nh_local_.advertise<nav_msgs::Odometry>("odometry", 1);
    pose_pub_ = nh_local_.advertise<geometry_msgs::PoseStamped>("pose", 1);
    info_pub_ = nh_local_.advertise<FovisInfo>("info", 1);
    features_pub_ = it_.advertise("features", 1);
  }

  virtual ~OdometerBase()
  {
    if (visual_odometer_) delete visual_odometer_;
    if (rectification_) delete rectification_;
  }

  const fovis::VisualOdometryOptions& getOptions() const
  {
    return visual_odometer_options_;
  }

  /**
   * Sets the depth source, must be called once before calling process()
   */
  void setDepthSource(fovis::DepthSource* source)
  {
    depth_source_ = source;
  }

  static void rosToFovis(const image_geometry::PinholeCameraModel& camera_model,
      fovis::CameraIntrinsicsParameters& parameters)
  {
    parameters.cx = camera_model.cx();
    parameters.cy = camera_model.cy();
    parameters.fx = camera_model.fx();
    parameters.fy = camera_model.fy();
    parameters.width = camera_model.reducedResolution().width;
    parameters.height = camera_model.reducedResolution().height;
  }

  /**
   * To be called by implementing classes after the depth source has
   * been fed with data.
   */
  void process(
      const sensor_msgs::ImageConstPtr& image_msg, 
      const sensor_msgs::CameraInfoConstPtr& info_msg)
  {
    ros::WallTime start_time = ros::WallTime::now();

    bool first_run = false;
    if (visual_odometer_ == NULL)
    {
      first_run = true;
      initOdometer(info_msg);
    }
    ROS_ASSERT(visual_odometer_ != NULL);
    ROS_ASSERT(depth_source_ != NULL);

    // convert image if necessary
    uint8_t *image_data;
    cv_bridge::CvImageConstPtr cv_ptr = 
      cv_bridge::toCvShare(image_msg, sensor_msgs::image_encodings::MONO8);
    image_data = cv_ptr->image.data;
    int step = cv_ptr->image.step[0];

    ROS_ASSERT(step == static_cast<int>(image_msg->width));

    // pass image to odometer
    visual_odometer_->processFrame(image_data, depth_source_);

    // skip visualization on first run as no reference image is present
    if (!first_run && features_pub_.getNumSubscribers() > 0)
    {
      cv_bridge::CvImage cv_image;
      cv_image.header.stamp = image_msg->header.stamp;
      cv_image.header.frame_id = image_msg->header.frame_id;
      cv_image.encoding = sensor_msgs::image_encodings::BGR8;
      cv_image.image = visualization::paint(visual_odometer_);
      features_pub_.publish(cv_image.toImageMsg());
    }

    // create odometry and pose messages
    odom_msg_.header.stamp = image_msg->header.stamp;
    odom_msg_.header.frame_id = odom_frame_id_;
    odom_msg_.child_frame_id = base_link_frame_id_;
    
    pose_msg_.header.stamp = image_msg->header.stamp;
    pose_msg_.header.frame_id = base_link_frame_id_;

    // on success, start fill message and tf
    fovis::MotionEstimateStatusCode status = 
      visual_odometer_->getMotionEstimateStatus();
    if (status == fovis::SUCCESS)
    {
      // get pose and motion from odometer
      const Eigen::Isometry3d& pose = visual_odometer_->getPose();
      tf::Transform sensor_pose;
      eigenToTF(pose, sensor_pose);
      // calculate transform of odom to base based on base to sensor 
      // and sensor to sensor
      tf::StampedTransform current_base_to_sensor;
      getBaseToSensorTransform(
          image_msg->header.stamp, image_msg->header.frame_id, 
          current_base_to_sensor);
      tf::Transform base_transform = 
        initial_base_to_sensor_ * sensor_pose * current_base_to_sensor.inverse();

      // publish transform
      if (publish_tf_)
      {
        tf_broadcaster_.sendTransform(
            tf::StampedTransform(base_transform, image_msg->header.stamp,
            odom_frame_id_, base_link_frame_id_));
      }

      // fill odometry and pose msg
      tf::poseTFToMsg(base_transform, odom_msg_.pose.pose);
      pose_msg_.pose = odom_msg_.pose.pose;

      // can we calculate velocities?
      double dt = last_time_.isZero() ? 
        0.0 : (image_msg->header.stamp - last_time_).toSec();
      if (dt > 0.0)
      {
        const Eigen::Isometry3d& motion = visual_odometer_->getMotionEstimate();
        tf::Transform sensor_motion;
        eigenToTF(motion, sensor_motion);
        // in theory the first factor would have to be base_to_sensor of t-1
        // and not of t (irrelevant for static base to sensor anyways)
        tf::Transform delta_base_transform = 
          current_base_to_sensor * sensor_motion * current_base_to_sensor.inverse();
        // calculate twist from delta transform
        odom_msg_.twist.twist.linear.x = delta_base_transform.getOrigin().getX() / dt;
        odom_msg_.twist.twist.linear.y = delta_base_transform.getOrigin().getY() / dt;
        odom_msg_.twist.twist.linear.z = delta_base_transform.getOrigin().getZ() / dt;
        tf::Quaternion delta_rot = delta_base_transform.getRotation();
        double angle = delta_rot.getAngle();
        tf::Vector3 axis = delta_rot.getAxis();
        tf::Vector3 angular_twist = axis * angle / dt;
        odom_msg_.twist.twist.angular.x = angular_twist.x();
        odom_msg_.twist.twist.angular.y = angular_twist.y();
        odom_msg_.twist.twist.angular.z = angular_twist.z();

        // add covariance
        const Eigen::MatrixXd& motion_cov = visual_odometer_->getMotionEstimateCov();
        for (int i=0;i<6;i++)
          for (int j=0;j<6;j++)
            odom_msg_.twist.covariance[j*6+i] = motion_cov(i,j);
      }
      // TODO integrate covariance for pose covariance
      last_time_ = image_msg->header.stamp;
    }
    else
    {
      // Previous messages with the current timestamp will be published
      ROS_WARN_STREAM("fovis odometry status: " << 
          fovis::MotionEstimateStatusCodeStrings[status]);
      last_time_ = ros::Time(0);
    }
    odom_pub_.publish(odom_msg_);
    pose_pub_.publish(pose_msg_);

    // create and publish fovis info msg
    FovisInfo fovis_info_msg;
    fovis_info_msg.header.stamp = image_msg->header.stamp;
    fovis_info_msg.change_reference_frame = 
      visual_odometer_->getChangeReferenceFrames();
    fovis_info_msg.fast_threshold =
      visual_odometer_->getFastThreshold();
    const fovis::OdometryFrame* frame = 
      visual_odometer_->getTargetFrame();
    fovis_info_msg.num_total_detected_keypoints =
      frame->getNumDetectedKeypoints();
    fovis_info_msg.num_total_keypoints = frame->getNumKeypoints();
    fovis_info_msg.num_detected_keypoints.resize(frame->getNumLevels());
    fovis_info_msg.num_keypoints.resize(frame->getNumLevels());
    for (int i = 0; i < frame->getNumLevels(); ++i)
    {
      fovis_info_msg.num_detected_keypoints[i] =
        frame->getLevel(i)->getNumDetectedKeypoints();
      fovis_info_msg.num_keypoints[i] =
        frame->getLevel(i)->getNumKeypoints();
    }
    const fovis::MotionEstimator* estimator = 
      visual_odometer_->getMotionEstimator();
    fovis_info_msg.motion_estimate_status_code =
      estimator->getMotionEstimateStatus();
    fovis_info_msg.motion_estimate_status = 
      fovis::MotionEstimateStatusCodeStrings[
        fovis_info_msg.motion_estimate_status_code];
    fovis_info_msg.num_matches = estimator->getNumMatches();
    fovis_info_msg.num_inliers = estimator->getNumInliers();
    fovis_info_msg.num_reprojection_failures =
      estimator->getNumReprojectionFailures();
    fovis_info_msg.motion_estimate_valid = 
      estimator->isMotionEstimateValid();
    ros::WallDuration time_elapsed = ros::WallTime::now() - start_time;
    fovis_info_msg.runtime = time_elapsed.toSec();
    info_pub_.publish(fovis_info_msg);
  }


private:

  /**
   * Initializes the visual odometry. 
   */
  void initOdometer(const sensor_msgs::CameraInfoConstPtr& info_msg)
  {
    // create rectification
    image_geometry::PinholeCameraModel model;
    model.fromCameraInfo(info_msg);
    fovis::CameraIntrinsicsParameters cam_params;
    rosToFovis(model, cam_params);
    fovis::Rectification* rectification = new fovis::Rectification(cam_params);

    // instanciate odometer
    visual_odometer_ = 
      new fovis::VisualOdometry(rectification, visual_odometer_options_);

    // store initial transform for later usage
    getBaseToSensorTransform(info_msg->header.stamp, 
        info_msg->header.frame_id,
        initial_base_to_sensor_);

    // print options
    std::stringstream info;
    info << "Initialized fovis odometry with the following options:\n";
    for (fovis::VisualOdometryOptions::iterator iter = visual_odometer_options_.begin();
        iter != visual_odometer_options_.end();
        ++iter)
    {
      std::string key = iter->first;
      std::replace(key.begin(), key.end(), '-', '_');
      info << key << " = " << iter->second << std::endl;
    }
    ROS_INFO_STREAM(info.str());
  }

  /**
   * Loads parameters from ROS node handle into members.
   */
  void loadParams()
  {
    nh_local_.param("odom_frame_id", odom_frame_id_, std::string("/odom"));
    nh_local_.param("base_link_frame_id", base_link_frame_id_, std::string("/base_link"));
    nh_local_.param("publish_tf", publish_tf_, true);

    for (fovis::VisualOdometryOptions::iterator iter = visual_odometer_options_.begin();
        iter != visual_odometer_options_.end();
        ++iter)
    {
      // NOTE: this only accepts parameters if given through
      // launch files with the argument "type" set to "string":
      //   e.g. <param name="fast_threshold_adaptive_gain" type="string" value="0.001"/>
      // Passing parameters through the command line does not work as rosparam
      // automagically treats numbers as int or float and we need strings here.
      std::string key = iter->first;
      // to comply with ROS standard of parameter naming
      std::replace(key.begin(), key.end(), '-', '_');
      if (nh_local_.hasParam(key))
      {
        std::string value;
        nh_local_.getParam(key, value);
        visual_odometer_options_[iter->first] = value;
      }
    }
  }

  void getBaseToSensorTransform(const ros::Time& stamp, 
      const std::string& sensor_frame_id, tf::StampedTransform& base_to_sensor)
  {
    std::string error_msg;
    if (tf_listener_.canTransform(
          base_link_frame_id_, sensor_frame_id, stamp, &error_msg))
    {
      tf_listener_.lookupTransform(
          base_link_frame_id_,
          sensor_frame_id,
          stamp, base_to_sensor);
    }
    else
    {
      ROS_WARN_THROTTLE(10.0, "The tf from '%s' to '%s' does not seem to be "
                              "available, will assume it as identity!", 
                              base_link_frame_id_.c_str(),
                              sensor_frame_id.c_str());
      ROS_DEBUG("Transform error: %s", error_msg.c_str());
      base_to_sensor.setIdentity();
    }
  }

  void eigenToTF(const Eigen::Isometry3d& pose, tf::Transform& transform)
  {
    tf::Vector3 origin(
        pose.translation().x(), pose.translation().y(), pose.translation().z());
    Eigen::Quaterniond rotation(pose.rotation());
    tf::Quaternion quat(rotation.x(), rotation.y(),
        rotation.z(), rotation.w());
    transform = tf::Transform(quat, origin);
  }


private:

  fovis::VisualOdometry* visual_odometer_;
  fovis::Rectification* rectification_;
  fovis::DepthSource* depth_source_;
  fovis::VisualOdometryOptions visual_odometer_options_;

  ros::Time last_time_;

  // tf related
  std::string sensor_frame_id_;
  std::string odom_frame_id_;
  std::string base_link_frame_id_;
  bool publish_tf_;
  tf::StampedTransform initial_base_to_sensor_;
  tf::TransformListener tf_listener_;
  tf::TransformBroadcaster tf_broadcaster_;
  
  // Messages
  nav_msgs::Odometry odom_msg_;
  geometry_msgs::PoseStamped pose_msg_;

  ros::NodeHandle nh_local_;

  // publisher
  ros::Publisher odom_pub_;
  ros::Publisher pose_pub_;
  ros::Publisher info_pub_;
  image_transport::Publisher features_pub_;
  image_transport::ImageTransport it_;
};

} // end of namespace

