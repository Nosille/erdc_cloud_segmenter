/** ROS node 

/* 

Copyright (c) 2017

*/

#ifndef TRACKER_H
#define TRACKER_H

#include <mutex>
#include <memory>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/message_filter.h>
#include <tf2_eigen/tf2_eigen.h>

#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <pcl_conversions/pcl_conversions.h>

// #include <dynamic_reconfigure/server.h>
// #include <dynamic_reconfigure/client.h>
//#include <erdc_cloud_segmenter/trackerConfig.h>

#include "common/color.hpp"
#include "common/bounding_box.hpp"
#include "common/parameter.hpp"  // common::getSegmenterParams
#include "common/publisher.hpp"  // common::publishCloud
#include "common/time.hpp"       // common::Clock
#include "common/types/type.h"   // PointICloudPtr
#include "common/types/object.hpp"
#include "object_builders/object_builder_manager.hpp"
#include "segmenters/segmenter_manager.hpp"  // segmenter::createGroundSegmenter
#include <common/msgs/autosense_msgs/PointCloud2Array.h>
#include "tracking/tracking_worker_manager.hpp"
#include "common/msgs/autosense_msgs/TrackingObjectArray.h"
#include "common/msgs/autosense_msgs/TrackingFixedTrajectoryArray.h"

namespace ERDC_Cloud_Segmenter
{
  class Tracker
  {
  public:
    Tracker(const ros::NodeHandle &node_handle, const ros::NodeHandle &private_node_handle);

    /** Destructor */
    ~Tracker();

    /** 
     * Main run loop
     */
    virtual void onInit();

    //! Callback
    //void reconfigure_server_callback(erdc_cloud_segmenter::trackerConfig &config, uint32_t level);
    void input_callback(const autosense_msgs::PointCloud2ArrayConstPtr& msg);
    static bool getSensorPose(const tf2_ros::Buffer& tf_buffer,
                              const std::string& source_frame,
                              const std::string& target_frame,
                              const ros::Time& query_time,
                              Eigen::Matrix4d* trans);

    // Methods


    //Variables
    std::string node_name{"tracker_node"};
    

  private:
    // ROS 
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    
    ros::Subscriber sub_input_;
    
    ros::Publisher pub_segments_coarse_;
    ros::Publisher pub_segments_predict_;
    ros::Publisher pub_segments_;
    ros::Publisher pub_tracking_objects_;
    ros::Publisher pub_tracking_objects_cloud_;
    ros::Publisher pub_tracking_objects_velocity_;
    ros::Publisher pub_tracking_objects_trajectory_;
    ros::Publisher pub_tracking_output_objects_;
    ros::Publisher pub_tracking_output_trajectories_;

    //erdc_cloud_segmenter::trackerConfig trackerConfig_;
    //boost::shared_ptr<dynamic_reconfigure::Server<erdc_cloud_segmenter::trackerConfig> > drServer_;
    //bool received_trackerConfig_;

    autosense::TrackingWorkerParams tracking_params_;
    std::unique_ptr<autosense::object_builder::BaseObjectBuilder> object_builder_ = nullptr;
    std::unique_ptr<autosense::tracking::BaseTrackingWorker> tracking_worker_ = nullptr;

    // Methods
    
    //Variables
    std::string map_frame_id_, sensor_frame_id_;
    std::string input_topic_;

    std::string pub_segments_coarse_topic_, pub_segments_predict_topic_, pub_segments_topic_;
    std::string pub_tracking_objects_topic_, pub_tracking_objects_cloud_topic_, pub_tracking_objects_velocity_topic_, pub_tracking_objects_trajectory_topic_;
    std::string pub_output_objects_topic_, pub_output_trajectories_topic_;

    double threshold_contian_IoU_ = 0.0;

    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_;
    
    float wait_for_tf_delay_;

    boost::recursive_mutex drServer_mutex_;

  }; // class Segmenter
} // namespace ERDC_Cloud_Segmenter

#endif  // TRACKER_H

