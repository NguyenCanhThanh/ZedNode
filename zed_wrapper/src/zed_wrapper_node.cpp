#include <nodelet/loader.h>
#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "zed_camera_node");

  // Start the ZED Nodelet
  nodelet::Loader nodelet;
  nodelet::M_string remap(ros::names::getRemappings());
  nodelet::V_string nargv;
  nodelet.load(ros::this_node::getName(), "zed_nodelets/ZEDWrapperNodelet", remap, nargv);

  ros::spin();

  return 0;
}
