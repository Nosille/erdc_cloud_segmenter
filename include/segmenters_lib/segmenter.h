/** ROS node 

/* 

Copyright (c) 2017

*/

#ifndef SEGMENTER_H
#define SEGMENTER_H

//#include "sensor_fusion.h"

#include <mutex>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/message_filter.h>
#include <tf2_eigen/tf2_eigen.h>

#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_representation.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <dynamic_reconfigure/server.h>
// #include <dynamic_reconfigure/client.h>
#include <erdc_cloud_segmenter/segmenterConfig.h>

#include "common/color.hpp"
#include "common/parameter.hpp"  // common::getSegmenterParams
#include "common/publisher.hpp"  // common::publishCloud
#include "common/time.hpp"       // common::Clock
#include "common/types/type.h"   // PointICloudPtr
#include "object_builders/object_builder_manager.hpp"
#include "segmenters/segmenter_manager.hpp"  // segmenter::createGroundSegmenter
#include <common/msgs/autosense_msgs/PointCloud2Array.h>
#include "roi_filters/roi.hpp"  // roi::applyROIFilter

// #include <pcl/registration/icp.h>
// #include <pcl/registration/icp_nl.h>
// #include <pcl/registration/transforms.h>

#ifdef _OPENMP
#include <pcl/features/normal_3d_omp.h>
#endif

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

namespace ERDC_Cloud_Segmenter
{
  class Segmenter
  {
  public:
    Segmenter(const ros::NodeHandle &node_handle, const ros::NodeHandle &private_node_handle);

    /** Destructor */
    ~Segmenter();

    /** 
     * Main run loop
     */
    virtual void onInit();

    //! Callback
    void reconfigure_server_callback(erdc_cloud_segmenter::segmenterConfig &config, uint32_t level);
    void input_callback(const sensor_msgs::PointCloud2::ConstPtr& msg);

    // Methods


    //Variables
    std::string node_name{"segmenter_node"};
    

  private:
    // ROS 
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    
    ros::Subscriber sub_input_;
    
    ros::Publisher pub_ground_;
    ros::Publisher pub_nonground_;
    ros::Publisher pub_segmented_;

    erdc_cloud_segmenter::segmenterConfig segmenterConfig_;
    boost::shared_ptr<dynamic_reconfigure::Server<erdc_cloud_segmenter::segmenterConfig> > drServer_;
    bool received_segmenterConfig_;

    autosense::ROIParams params_roi_;
    std::unique_ptr<autosense::segmenter::BaseSegmenter> ground_segmenter_;
    std::unique_ptr<autosense::segmenter::BaseSegmenter> nonground_segmenter_;

    // Methods
    void sorFilter(pcl::PCLPointCloud2::Ptr& cloud);

    //Variables
    std::string map_frame_id_, sensor_frame_id_;
    std::string input_topic_, ground_topic_, nonground_topic_, clusters_topic_;
    std::string ground_segmenter_type_, nonground_segmenter_type_;

    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_;
    
    float wait_for_tf_delay_;

    boost::recursive_mutex drServer_mutex_;

  }; // class Segmenter
} // namespace ERDC_Cloud_Segmenter

#endif  // Segmenter_H

