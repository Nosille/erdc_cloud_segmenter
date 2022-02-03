#include <nodelet/nodelet.h>
#include "segmenter.h"

namespace ERDC_Cloud_Segmenter{
class Segmenter_Nodelet: public nodelet::Nodelet
 {

 public:
    Segmenter_Nodelet(){}
    ~Segmenter_Nodelet(){}
    virtual void onInit();
    boost::shared_ptr<ERDC_Cloud_Segmenter::Segmenter> inst_;

    };

}