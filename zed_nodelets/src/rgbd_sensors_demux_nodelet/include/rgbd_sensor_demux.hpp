#ifndef RGBD_SENSOR_DEMUX_HPP
#define RGBD_SENSOR_DEMUX_HPP

#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <ros/subscriber.h>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include "zed_interfaces/RGBDSensors.h"

namespace zed_nodelets
{
class RgbdSensorsDemuxNodelet : public nodelet::Nodelet
{
public:
  RgbdSensorsDemuxNodelet();
  virtual ~RgbdSensorsDemuxNodelet();

protected:
  /*! \brief Initialization function called by the Nodelet base class
   */
  virtual void onInit();

  /*! \brief Callback for full topics synchronization
   */
  void msgCallback(const zed_interfaces::RGBDSensorsPtr& msg);

private:
  // Node handlers
  ros::NodeHandle mNh;   // Node handler
  ros::NodeHandle mNhP;  // Private Node handler

  // Publishers
  image_transport::CameraPublisher mPubRgb;
  image_transport::CameraPublisher mPubDepth;
  ros::Publisher mPubIMU;
  ros::Publisher mPubMag;

  // Subscribers
  ros::Subscriber mSub;
};

}  // namespace zed_nodelets

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(zed_nodelets::RgbdSensorsDemuxNodelet, nodelet::Nodelet)

#endif  // RGBD_SENSOR_DEMUX_HPP
