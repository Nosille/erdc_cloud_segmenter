#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include "segmenters_lib/tracker_nodelet.h"

namespace Cloud_Segmentor
{

 void Tracker_Nodelet::onInit()
 {

  NODELET_DEBUG("Initializing nodelet");
  inst_.reset(new Tracker(getMTNodeHandle(), getMTPrivateNodeHandle()));
 }

}

PLUGINLIB_EXPORT_CLASS(Cloud_Segmentor::Tracker_Nodelet, nodelet::Nodelet)
