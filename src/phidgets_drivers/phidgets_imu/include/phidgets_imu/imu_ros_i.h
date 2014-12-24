#ifndef PHIDGETS_IMU_IMU_ROS_I_H
#define PHIDGETS_IMU_IMU_ROS_I_H

#include <ros/ros.h>
#include <ros/service_server.h>
#include <boost/thread/mutex.hpp>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/Imu.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <phidgets_api/imu.h>

namespace phidgets {

const float G = 9.81;

class ImuRosI : public Imu
{
  typedef sensor_msgs::Imu              ImuMsg;
  typedef geometry_msgs::Vector3Stamped MagMsg;

  public:

    ImuRosI(ros::NodeHandle nh, ros::NodeHandle nh_private);

    bool calibrateService(std_srvs::Empty::Request  &req,
                          std_srvs::Empty::Response &res);

  private:

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::Publisher  imu_publisher_;
    ros::Publisher  mag_publisher_;
    ros::Publisher  cal_publisher_;
    ros::ServiceServer cal_srv_;

    bool initialized_;
    boost::mutex mutex_;
    ros::Time last_imu_time_;

    ImuMsg imu_msg_;

    ros::Time time_zero_;

    // params

    std::string frame_id_;
    int period_;  // rate in ms

    double angular_velocity_stdev_;
    double linear_acceleration_stdev_;

    // compass correction params (see http://www.phidgets.com/docs/1044_User_Guide)
    double cc_mag_field_;
    double cc_offset0_;
    double cc_offset1_;
    double cc_offset2_;
    double cc_gain0_;
    double cc_gain1_;
    double cc_gain2_;
    double cc_T0_;
    double cc_T1_;
    double cc_T2_;
    double cc_T3_;
    double cc_T4_;
    double cc_T5_;

    void calibrate();
    void initDevice();
    void dataHandler(CPhidgetSpatial_SpatialEventDataHandle* data, int count);
    void processImuData(CPhidgetSpatial_SpatialEventDataHandle* data, int i);
};

} //namespace phidgets

#endif // PHIDGETS_IMU_IMU_ROS_I_H
