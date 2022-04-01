
#include <typeinfo>
#include "segmenters_lib/segmenter.h"

#include <segmenters/segmenter_manager.hpp>  // segmenter::createGroundSegmenter


namespace ERDC_Cloud_Segmenter
{
  Segmenter::Segmenter(const ros::NodeHandle &node_handle, 
                        const ros::NodeHandle &private_node_handle) 
  // Initialization list
  :nh_(node_handle),
  pnh_(private_node_handle),
  wait_for_tf_delay_(0.1),
  received_segmenterConfig_(false),
  tfListener_(tfBuffer_)
  {
    this->onInit();
  }
  Segmenter::~Segmenter()
  {
    // pass
  }

  void Segmenter::onInit()
  {
    ROS_INFO_STREAM_NAMED(node_name.c_str(), "Begin segmenter");
    const std::string param_ns_prefix_ = "detect";  // NOLINT

  // ROS Parameters
    bool use_roi_filter;
    bool use_ground_segmenter;
    bool use_nonground_segmenter;

    // pnh_.param<std::string>("map_frame_id", map_frame_id_, "map");
    //   ROS_INFO_STREAM_NAMED(node_name, "map_frame_id set to " << map_frame_id_);

    // pnh_.param<std::string>("sensor_frame_id", sensor_frame_id_, "base_link");
    //   ROS_INFO_STREAM_NAMED(node_name, "sensor_frame_id set to " << sensor_frame_id_);
   
    nh_.param("wait_for_tf_delay", wait_for_tf_delay_, 1.0f);
      ROS_INFO_STREAM_NAMED(node_name, "wait_for_tf_delay set to " << wait_for_tf_delay_);

    nh_.param<std::string>("input_topic", input_topic_, "points");
      ROS_INFO_STREAM_NAMED(node_name, "input_topic set to " << input_topic_);

    nh_.param<std::string>("clusters_topic", clusters_topic_,"clusters");
      ROS_INFO_STREAM_NAMED(node_name, "clusters_topic set to " << clusters_topic_);

    nh_.param<std::string>(param_ns_prefix_+"/ground_topic", ground_topic_, "ground");
      ROS_INFO_STREAM_NAMED(node_name, param_ns_prefix_+"/ground_topic set to " << ground_topic_);                  

    nh_.param<std::string>(param_ns_prefix_+"/nonground_topic", nonground_topic_, "nonground");
      ROS_INFO_STREAM_NAMED(node_name, param_ns_prefix_+"/nonground_topic set to " << nonground_topic_);

    nh_.param<std::string>(param_ns_prefix_+"/ground_segmenter_type", ground_segmenter_type_, "GroundPlaneFittingSegmenter");
      ROS_INFO_STREAM_NAMED(node_name, param_ns_prefix_+"/ground_segmenter_type set to " << ground_segmenter_type_);
    nh_.param<std::string>(param_ns_prefix_+"/nonground_segmenter_type", nonground_segmenter_type_, "RegionEuclideanSegmenter");
      ROS_INFO_STREAM_NAMED(node_name, param_ns_prefix_+"/nonground_segmenter_type set to " << nonground_segmenter_type_);

    nh_.param(param_ns_prefix_+"/use_roi_filter", use_roi_filter, false);
      ROS_INFO_STREAM_NAMED(node_name, param_ns_prefix_+"/use_roi_filter set to " << use_roi_filter);
    nh_.param(param_ns_prefix_+"/use_ground_segmenter", use_ground_segmenter, false);
      ROS_INFO_STREAM_NAMED(node_name, param_ns_prefix_+"/use_ground_segmenter set to " << use_ground_segmenter);
    nh_.param(param_ns_prefix_+"/use_nonground_segmenter", use_nonground_segmenter, false);
      ROS_INFO_STREAM_NAMED(node_name, param_ns_prefix_+"/use_nonground_segmenter set to " << use_nonground_segmenter);


    params_roi_ = autosense::common::getRoiParams(nh_, param_ns_prefix_);

    autosense::SegmenterParams param = autosense::common::getSegmenterParams(nh_, param_ns_prefix_);
    param.segmenter_type = ground_segmenter_type_;
    ground_segmenter_ = autosense::segmenter::createGroundSegmenter(param);
    param.segmenter_type = nonground_segmenter_type_;
    nonground_segmenter_ = autosense::segmenter::createNonGroundSegmenter(param);
 

  //Setup Dynamic Reconfigure Server
    dynamic_reconfigure::Server<erdc_cloud_segmenter::segmenterConfig>::CallbackType 
        drServerCallback_ = boost::bind(&Segmenter::reconfigure_server_callback, this, _1, _2);
    drServer_.reset(new dynamic_reconfigure::Server<erdc_cloud_segmenter::segmenterConfig>(drServer_mutex_, nh_));
    drServer_->setCallback(drServerCallback_);
    
    //Wait on dyanamic param server to intialize values
    while(!received_segmenterConfig_)
    {
      ros::Duration(1.0).sleep();
      ROS_INFO_STREAM_NAMED(node_name, "Waiting on dynamic parameters.");
      ros::spinOnce();
    }
    
    //Push config values to parameter server
    segmenterConfig_.use_roi_filter = use_roi_filter;
    segmenterConfig_.use_ground_segmenter = use_ground_segmenter;
    segmenterConfig_.use_nonground_segmenter = use_nonground_segmenter;
    
    drServer_->updateConfig(segmenterConfig_);

  // ROS publishers and subscribers
    //Pointcloud subscribers
    sub_input_ = nh_.subscribe<sensor_msgs::PointCloud2>(input_topic_, 1, &Segmenter::input_callback, this);


    //Pointcloud publishers
    pub_ground_ = nh_.advertise<sensor_msgs::PointCloud2>(ground_topic_,1);
    pub_nonground_ = nh_.advertise<sensor_msgs::PointCloud2>(nonground_topic_,1);
    pub_segmented_ = nh_.advertise<autosense_msgs::PointCloud2Array>(clusters_topic_, 1);
  }

  void Segmenter::reconfigure_server_callback(erdc_cloud_segmenter::segmenterConfig &config, uint32_t level) 
  {
    if (received_segmenterConfig_)
    {
      ROS_INFO_STREAM("Reconfigure Request: " << "\n" << 
            "use_roi_filter: "                  << config.use_roi_filter << "\n" <<
            "use_nonground_segmenter: "         << config.use_nonground_segmenter);
    }
    segmenterConfig_ = config;
    received_segmenterConfig_ = true;

  }
  
  void Segmenter::input_callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    ros::WallTime begin = ros::WallTime::now();

  //kickout if no subscribers
    if(pub_ground_.getNumSubscribers()<=0 && 
       pub_nonground_.getNumSubscribers()<=0 &&
       pub_segmented_.getNumSubscribers()<=0) return; 

    autosense::PointCloud2Ptr cloud_pc2(new autosense::PointCloud2);
    pcl_conversions::toPCL(*msg, *cloud_pc2);
    ROS_DEBUG_STREAM(" Cloud inputs: " << (cloud_pc2->height*cloud_pc2->width) << " Points");

    std_msgs::Header header = msg->header;

  //ROI filter
    if (segmenterConfig_.use_roi_filter) 
    {
        autosense::roi::applyROIFilter(params_roi_, cloud_pc2);
    }
    ROS_DEBUG_STREAM(" After ROIFilter: " << (cloud_pc2->height*cloud_pc2->width) << " Points");


  //Ground Segmenter
    //Segment
    std::vector<pcl::PointIndices> clusters_indices;
    autosense::PointCloud2Ptr cloud_ground(new autosense::PointCloud2);
    autosense::PointCloud2Ptr cloud_nonground(new autosense::PointCloud2);

    if (segmenterConfig_.use_ground_segmenter) 
    {
      autosense::PointICloudPtr cloud(new autosense::PointICloud);
      pcl::fromPCLPointCloud2(*cloud_pc2, *cloud);
      ground_segmenter_->segment(*cloud, clusters_indices);
      pcl::PointIndices::Ptr ground_indices(new pcl::PointIndices);
      *ground_indices = clusters_indices[0];

      if (ground_indices->indices.size() > 0) {
        pcl::ExtractIndices<autosense::PointCloud2> indiceExtractor;
        indiceExtractor.setInputCloud(cloud_pc2);
        indiceExtractor.setIndices(ground_indices);

        // extract ground points
        indiceExtractor.setNegative(false);
        indiceExtractor.filter(*cloud_ground);

        // extract non-ground points
        indiceExtractor.setNegative(true);
        indiceExtractor.filter(*cloud_nonground);

        sorFilter(cloud_nonground);
      }

      //Convert to sensor_msgs and publish ground
      if(pub_ground_.getNumSubscribers()>0) 
      {
        ROS_DEBUG_STREAM(" Cloud Ground: " << (cloud_ground->height*cloud_ground->width) << " Points");
        sensor_msgs::PointCloud2::Ptr ground_out(new sensor_msgs::PointCloud2);  //nodelets require the published msg to be a pointer
        autosense::common::publishCloud(pub_ground_, header, *cloud_ground);
      }
    }
    else 
    {
      *cloud_nonground = *cloud_pc2;
    }
   
    //Convert to sensor_msgs and publish nonground
    if(pub_nonground_.getNumSubscribers()>0) 
    {
      ROS_DEBUG_STREAM(" Cloud NonGround: " << (cloud_nonground->height*cloud_nonground->width) << " Points");
      sensor_msgs::PointCloud2::Ptr nonground_out(new sensor_msgs::PointCloud2);  //nodelets require the published msg to be a pointer
      autosense::common::publishCloud(pub_nonground_, header, *cloud_nonground);
    }

  //Nonground segmenter
    std::vector<autosense::PointCloud2> cloud_clusters;
    if (segmenterConfig_.use_nonground_segmenter  && pub_segmented_.getNumSubscribers()>0) 
    {
      // reset clusters
      clusters_indices.clear();
      //publish cloud array
      autosense::PointICloudPtr cloud(new autosense::PointICloud);
      pcl::fromPCLPointCloud2(*cloud_nonground, *cloud);
      nonground_segmenter_->segment(*cloud, clusters_indices);
      for(int i; i<clusters_indices.size(); i++)
      {
        
        // extract points
        pcl::ExtractIndices<autosense::PointCloud2> indiceExtractor;
        pcl::PointIndices::Ptr current_indices(new pcl::PointIndices);
        *current_indices = clusters_indices[i];
        indiceExtractor.setInputCloud(cloud_nonground);
        indiceExtractor.setIndices(current_indices);
        indiceExtractor.setNegative(false);
        autosense::PointCloud2 current_cluster;
        indiceExtractor.filter(current_cluster);
        cloud_clusters.push_back(current_cluster);
      }
      autosense::common::publishPointCloudArray(pub_segmented_, header, cloud_clusters);
    }
    
    ros::WallDuration duration = ros::WallTime::now() - begin;
    ROS_DEBUG_STREAM_NAMED(node_name, "Segmentation took " << duration.toSec() << "sec");
  }

/**
   * @brief @b StatisticalOutlierRemoval uses point neighborhood statistics to filter outlier data.
  * \details The algorithm iterates through the entire input twice:
  * During the first iteration it will compute the average distance that each point has to its nearest k neighbors.
  * The value of k can be set using setMeanK().
  * Next, the mean and standard deviation of all these distances are computed in order to determine a distance threshold.
  * The distance threshold will be equal to: mean + stddev_mult * stddev.
  * The multiplier for the standard deviation can be set using setStddevMulThresh().
  * During the next iteration the points will be classified as inlier or outlier if their average neighbor distance is below or above this threshold respectively.
  * <br>
  * The neighbors found for each query point will be found amongst ALL points of setInputCloud(), not just those indexed by setIndices().
  * The setIndices() method only indexes the points that will be iterated through as search query points.
  * <br><br>
  * For more information:
  *   - R. B. Rusu, Z. C. Marton, N. Blodow, M. Dolha, and M. Beetz.
  *     Towards 3D Point Cloud Based Object Maps for Household Environments
  *     Robotics and Autonomous Systems Journal (Special Issue on Semantic Knowledge), 2008.
  * <br><br>
   * 
   * @param cloud
   */
  void Segmenter::sorFilter(pcl::PCLPointCloud2::Ptr& cloud)
  {
    ros::WallTime filter_time = ros::WallTime::now();

    if (segmenterConfig_.sor_filter)
    {
      pcl::StatisticalOutlierRemoval<pcl::PCLPointCloud2> sor;
      sor.setInputCloud (cloud);
      sor.setMeanK (segmenterConfig_.sor_meanK);
      sor.setStddevMulThresh (segmenterConfig_.sor_stddevMulThresh);
      sor.filter (*cloud);

      ros::WallDuration filter_duration = ros::WallTime::now() - filter_time;
      ROS_DEBUG_STREAM_NAMED(node_name, "Sor Filter took: " << filter_duration.toSec() << "sec");
    }
  }
} // namespace ERDC_Cloud_Segmenter

