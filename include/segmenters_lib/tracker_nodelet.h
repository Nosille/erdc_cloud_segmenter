#include <nodelet/nodelet.h>
#include "tracker.h"

namespace Cloud_Segmentor{
class Tracker_Nodelet: public nodelet::Nodelet
 {

 public:
    Tracker_Nodelet(){}
    ~Tracker_Nodelet(){}
    virtual void onInit();
    boost::shared_ptr<Cloud_Segmentor::Tracker> inst_;

    };

}