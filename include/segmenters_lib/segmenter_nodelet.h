#include <nodelet/nodelet.h>
#include "segmenter.h"

namespace Cloud_Segmentor{
class Segmenter_Nodelet: public nodelet::Nodelet
 {

 public:
    Segmenter_Nodelet(){}
    ~Segmenter_Nodelet(){}
    virtual void onInit();
    boost::shared_ptr<Cloud_Segmentor::Segmenter> inst_;

    };

}