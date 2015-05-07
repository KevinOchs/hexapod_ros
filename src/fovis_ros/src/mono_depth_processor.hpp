#ifndef STEREO_PROCESSOR_H_
#define STEREO_PROCESSOR_H_

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/subscriber_filter.h>

namespace fovis_ros
{

/**
 * This is an abstract base class for nodes that process RGBD data, 
 * such as Microsoft Kinect.
 * It handles synchronization of input topics (approximate or exact)
 * and checks for sync errors.
 * To use this class, subclass it and implement the imageCallback() method.
 */
class MonoDepthProcessor
{

private:

  // subscriber
  image_transport::SubscriberFilter image_sub_, depth_sub_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> image_info_sub_, depth_info_sub_;
  typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> ExactPolicy;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> ApproximatePolicy;
  typedef message_filters::Synchronizer<ExactPolicy> ExactSync;
  typedef message_filters::Synchronizer<ApproximatePolicy> ApproximateSync;
  boost::shared_ptr<ExactSync> exact_sync_;
  boost::shared_ptr<ApproximateSync> approximate_sync_;
  int queue_size_;

  // for sync checking
  ros::WallTimer check_synced_timer_;
  int image_received_, depth_received_, image_info_received_, depth_info_received_, all_received_;

  // for sync checking
  static void increment(int* value)
  {
    ++(*value);
  }

  void dataCb(const sensor_msgs::ImageConstPtr& image_msg,
              const sensor_msgs::ImageConstPtr& depth_image_msg,
              const sensor_msgs::CameraInfoConstPtr& image_info_msg,
              const sensor_msgs::CameraInfoConstPtr& depth_info_msg)
  {
 
    // For sync error checking
    ++all_received_; 

    // call implementation
    imageCallback(image_msg, depth_image_msg, image_info_msg, depth_info_msg);
  }

  void checkInputsSynchronized()
  {
    int threshold = 3 * all_received_;
    if (image_received_ >= threshold || depth_received_ >= threshold || 
        image_info_received_ >= threshold || depth_info_received_ >= threshold) {
      ROS_WARN("[stereo_processor] Low number of synchronized image/depth/image_info/depth_info tuples received.\n"
               "Images received:            %d (topic '%s')\n"
               "Depth images received:      %d (topic '%s')\n"
               "Image camera info received: %d (topic '%s')\n"
               "Depth camera info received: %d (topic '%s')\n"
               "Synchronized tuples: %d\n",
               image_received_, image_sub_.getTopic().c_str(),
               depth_received_, depth_sub_.getTopic().c_str(),
               image_info_received_, image_info_sub_.getTopic().c_str(),
               depth_info_received_, depth_info_sub_.getTopic().c_str(),
               all_received_);
    }
  }


protected:

  /**
   * Constructor, subscribes to input topics using image transport and registers
   * callbacks.
   * \param transport The image transport to use
   */
  MonoDepthProcessor(const std::string& transport) :
    image_received_(0), depth_received_(0), image_info_received_(0), depth_info_received_(0), all_received_(0)
  {
    // Read local parameters
    ros::NodeHandle local_nh("~");

    // Resolve topic names
    ros::NodeHandle nh;
    std::string camera_ns = nh.resolveName("camera");
    std::string image_topic = ros::names::clean(camera_ns + "/rgb/image_rect");
    std::string depth_topic = ros::names::clean(camera_ns + "/depth_registered/image_rect");

    std::string image_info_topic = camera_ns + "/rgb/camera_info";
    std::string depth_info_topic = camera_ns + "/depth_registered/camera_info";

    // Subscribe to four input topics.
    ROS_INFO("Subscribing to:\n\t* %s\n\t* %s\n\t* %s\n\t* %s", 
        image_topic.c_str(), depth_topic.c_str(),
        image_info_topic.c_str(), depth_info_topic.c_str());

    image_transport::ImageTransport it(nh);
    image_sub_.subscribe(it, image_topic, 1, transport);
    depth_sub_.subscribe(it, depth_topic, 1, transport);
    image_info_sub_.subscribe(nh, image_info_topic, 1);
    depth_info_sub_.subscribe(nh, depth_info_topic, 1);

    // Complain every 15s if the topics appear unsynchronized
    image_sub_.registerCallback(boost::bind(MonoDepthProcessor::increment, &image_received_));
    depth_sub_.registerCallback(boost::bind(MonoDepthProcessor::increment, &depth_received_));
    image_info_sub_.registerCallback(boost::bind(MonoDepthProcessor::increment, &image_info_received_));
    depth_info_sub_.registerCallback(boost::bind(MonoDepthProcessor::increment, &depth_info_received_));
    check_synced_timer_ = nh.createWallTimer(ros::WallDuration(15.0),
                                             boost::bind(&MonoDepthProcessor::checkInputsSynchronized, this));

    // Synchronize input topics. Optionally do approximate synchronization.
    local_nh.param("queue_size", queue_size_, 5);
    bool approx;
    local_nh.param("approximate_sync", approx, true);
    if (approx)
    {
      approximate_sync_.reset(new ApproximateSync(ApproximatePolicy(queue_size_),
                                                  image_sub_, depth_sub_, image_info_sub_, depth_info_sub_) );
      approximate_sync_->registerCallback(boost::bind(&MonoDepthProcessor::dataCb, this, _1, _2, _3, _4));
    }
    else
    {
      exact_sync_.reset(new ExactSync(ExactPolicy(queue_size_),
                                      image_sub_, depth_sub_, image_info_sub_, depth_info_sub_) );
      exact_sync_->registerCallback(boost::bind(&MonoDepthProcessor::dataCb, this, _1, _2, _3, _4));
    }
  }

  /**
   * Implement this method in sub-classes 
   */
  virtual void imageCallback(const sensor_msgs::ImageConstPtr& image_msg,
                             const sensor_msgs::ImageConstPtr& depth_msg,
                             const sensor_msgs::CameraInfoConstPtr& image_info_msg,
                             const sensor_msgs::CameraInfoConstPtr& depth_info_msg) = 0;

};

} // end of namespace

#endif

