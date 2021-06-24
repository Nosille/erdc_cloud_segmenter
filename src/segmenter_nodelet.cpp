#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include "segmenters_lib/segmenter_nodelet.h"

namespace Cloud_Segmentor
{

 void Segmenter_Nodelet::onInit()
 {

  NODELET_DEBUG("Initializing nodelet");
  inst_.reset(new Segmenter(getMTNodeHandle(), getMTPrivateNodeHandle()));
 }

}

PLUGINLIB_EXPORT_CLASS(Cloud_Segmentor::Segmenter_Nodelet, nodelet::Nodelet)
