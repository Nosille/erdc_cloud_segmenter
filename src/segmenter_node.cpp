#include "segmenters_lib/segmenter.h"
#include <ros/callback_queue.h>

int main(int argc, char **argv)
{
  //Setup ros node
  std::string node_name = "segmenter_node";
  ros::init(argc, argv, node_name);
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  //Create Segmenter class
  Cloud_Segmentor::Segmenter node(nh, nh_private);

  //Setup blocking spinner for subscribers
  ros::MultiThreadedSpinner sub_spinner(0); // Use a thread for each core
  sub_spinner.spin(); // spin() will not return until the node has been shutdown

  return 0;
}