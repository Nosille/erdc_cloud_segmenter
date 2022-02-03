#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include "segmenters_lib/segmenter_nodelet.h"

namespace ERDC_Cloud_Segmenter
{

 void Segmenter_Nodelet::onInit()
 {

  NODELET_DEBUG("Initializing nodelet");
  inst_.reset(new Segmenter(getMTNodeHandle(), getMTPrivateNodeHandle()));
 }

}

PLUGINLIB_EXPORT_CLASS(ERDC_Cloud_Segmenter::Segmenter_Nodelet, nodelet::Nodelet)
