#include <nodelet/nodelet.h>
#include "tracker.h"

namespace ERDC_Cloud_Segmenter{
class Tracker_Nodelet: public nodelet::Nodelet
 {

 public:
    Tracker_Nodelet(){}
    ~Tracker_Nodelet(){}
    virtual void onInit();
    boost::shared_ptr<ERDC_Cloud_Segmenter::Tracker> inst_;

    };

}