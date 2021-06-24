
#include <typeinfo>
#include "segmenters_lib/tracker.h"

#include <segmenters/segmenter_manager.hpp>  // segmenter::createGroundSegmenter


namespace Cloud_Segmentor
{
  Tracker::Tracker(const ros::NodeHandle &node_handle, 
                        const ros::NodeHandle &private_node_handle) 
  // Initialization list
  :nh_(node_handle),
  pnh_(private_node_handle),
  wait_for_tf_delay_(0.1),
  //received_trackerConfig_(false),
  tfListener_(tfBuffer_)
  {
    this->onInit();
  }
  Tracker::~Tracker()
  {
    // pass
  }

  void Tracker::onInit()
  {
    ROS_INFO_STREAM_NAMED(node_name.c_str(), "Begin tracker");
    const std::string param_ns_prefix_ = "track";  // NOLINT

  // ROS Parameters
    bool use_roi_filter;
    bool use_nontracker;

    nh_.param<std::string>("map_frame_id", map_frame_id_, "map");
      ROS_INFO_STREAM_NAMED(node_name, "map_frame_id set to " << map_frame_id_);

    nh_.param("wait_for_tf_delay", wait_for_tf_delay_, wait_for_tf_delay_);
      ROS_INFO_STREAM_NAMED(node_name, "wait_for_tf_delay set to " << wait_for_tf_delay_);

    nh_.param<std::string>("clusters_topic", input_topic_, "points");
      ROS_INFO_STREAM_NAMED(node_name, "clusters_topic set to " << input_topic_);

    nh_.param<std::string>(param_ns_prefix_ + "/pub_segments_topic", pub_segments_topic_, "segments/fine");
      ROS_INFO_STREAM_NAMED(node_name, param_ns_prefix_ + "/pub_segments_topic set to " << pub_segments_topic_);
    nh_.param<std::string>(param_ns_prefix_ + "/pub_segments_coarse_topic", pub_segments_coarse_topic_, "segments/course");
      ROS_INFO_STREAM_NAMED(node_name, param_ns_prefix_ + "/pub_segments_course_topic set to " << pub_segments_coarse_topic_);
    nh_.param<std::string>(param_ns_prefix_ + "/pub_segments_predict_topic", pub_segments_predict_topic_, "segments/predict");
    ROS_INFO_STREAM_NAMED(node_name, param_ns_prefix_ + "/pub_segments_predict_topic set to " << pub_segments_predict_topic_);
    
    nh_.param<std::string>(param_ns_prefix_ + "/pub_tracking_objects_topic", pub_tracking_objects_topic_, "objects");
      ROS_INFO_STREAM_NAMED(node_name, param_ns_prefix_ + "/pub_tracking_objects_topic set to " << pub_tracking_objects_topic_);
    nh_.param<std::string>(param_ns_prefix_ + "/pub_tracking_objects_cloud_topic", pub_tracking_objects_cloud_topic_, "objects/cloud");
      ROS_INFO_STREAM_NAMED(node_name, param_ns_prefix_ + "/pub_tracking_objects_cloud_topic set to " << pub_tracking_objects_cloud_topic_);
    nh_.param<std::string>(param_ns_prefix_ + "/pub_tracking_objects_velocity_topic", pub_tracking_objects_velocity_topic_, "objects/velocity");
      ROS_INFO_STREAM_NAMED(node_name, param_ns_prefix_ + "/pub_tracking_objects_velocity_topic set to " << pub_tracking_objects_velocity_topic_);
    nh_.param<std::string>(param_ns_prefix_ + "/pub_tracking_objects_trajectory_topic", pub_tracking_objects_trajectory_topic_, "objects/trajectory");
      ROS_INFO_STREAM_NAMED(node_name, param_ns_prefix_ + "/pub_tracking_objects_trajectory_topic set to " << pub_tracking_objects_trajectory_topic_);

    nh_.param<std::string>(param_ns_prefix_ + "/pub_output_objects_topic", pub_output_objects_topic_, "output/objects");
      ROS_INFO_STREAM_NAMED(node_name, param_ns_prefix_ + "/pub_output_objects_topic set to " << pub_output_objects_topic_);
    nh_.param<std::string>(param_ns_prefix_ + "/pub_output_trajectories_topic", pub_output_trajectories_topic_, "output/trajectories");
      ROS_INFO_STREAM_NAMED(node_name, param_ns_prefix_ + "/pub_output_trajectories_topic set to " << pub_output_trajectories_topic_);

    nh_.param<double>(param_ns_prefix_ + "/threshold_contian_IoU", threshold_contian_IoU_, 1.0);

    autosense::TrackingWorkerParams tracking_params_;
    tracking_params_ = autosense::common::getTrackingWorkerParams(nh_, param_ns_prefix_);
    
    
    // Init core compoments
    object_builder_ = autosense::object_builder::createObjectBuilder();
    if (nullptr == object_builder_) 
    {
        ROS_FATAL("Failed to create object_builder_.");
        return;
    }
    tracking_worker_ = autosense::tracking::createTrackingWorker(tracking_params_);
    if (nullptr == tracking_worker_) 
    {
        ROS_FATAL("Failed to create tracking_worker_.");
        return;
    }

  //Setup Dynamic Reconfigure Server
    // dynamic_reconfigure::Server<cloud_segmentor::trackerConfig>::CallbackType 
    //     drServerCallback_ = boost::bind(&Tracker::reconfigure_server_callback, this, _1, _2);
    // drServer_.reset(new dynamic_reconfigure::Server<cloud_segmentor::trackerConfig>(drServer_mutex_, nh_));
    // drServer_->setCallback(drServerCallback_);
    
    //Wait on dyanamic param server to intialize values
    // while(!received_trackerConfig_)
    // {
    //   ros::Duration(1.0).sleep();
    //   ROS_INFO_STREAM_NAMED(node_name, "Waiting on dynamic parameters.");
    //   ros::spinOnce();
    // }
    
    //drServer_->updateConfig(trackerConfig_);

  // ROS publishers and subscribers
    //Pointcloud subscribers
    sub_input_ = nh_.subscribe<autosense_msgs::PointCloud2Array>(input_topic_, 10, &Tracker::input_callback, this);


    //Pointcloud publishers
    // segments
    pub_segments_ = nh_.advertise<visualization_msgs::MarkerArray>(pub_segments_topic_, 1);
    pub_segments_coarse_ = nh_.advertise<visualization_msgs::MarkerArray>(pub_segments_coarse_topic_, 1);
    pub_segments_predict_ = nh_.advertise<visualization_msgs::MarkerArray>(pub_segments_predict_topic_, 1);
    // tracking infos for debugging
    pub_tracking_objects_ = nh_.advertise<visualization_msgs::MarkerArray>(pub_tracking_objects_topic_, 1);
    pub_tracking_objects_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>(pub_tracking_objects_cloud_topic_, 1);
    pub_tracking_objects_velocity_ = nh_.advertise<visualization_msgs::MarkerArray>(pub_tracking_objects_velocity_topic_, 1);
    pub_tracking_objects_trajectory_ = nh_.advertise<visualization_msgs::MarkerArray>(pub_tracking_objects_trajectory_topic_, 1);
    // the whole tracking output
    pub_tracking_output_objects_ = nh_.advertise<autosense_msgs::TrackingObjectArray>(pub_output_objects_topic_, 1);
    pub_tracking_output_trajectories_ = nh_.advertise<autosense_msgs::TrackingFixedTrajectoryArray>(pub_output_trajectories_topic_, 1);
  }

  // void Tracker::reconfigure_server_callback(cloud_segmentor::trackerConfig &config, uint32_t level) 
  // {
  //   if (received_trackerConfig_)
  //   {
  //     ROS_INFO_STREAM("Reconfigure Request: " << "\n" << 
  //           "use_roi_filter: "                  << config.use_roi_filter << "\n" <<
  //           "use_nonground_segmenter: "         << config.use_nonground_segmenter);
  //   }
  //   trackerConfig_ = config;
  //   received_trackerConfig_ = true;

  // }
  
  void Tracker::input_callback(const autosense_msgs::PointCloud2ArrayConstPtr& msg)
  {
    ROS_INFO_STREAM("input_callback");
    ros::WallTime startTimer = ros::WallTime::now();
    ros::Time now = ros::Time::now();
    ROS_INFO("Clusters size: %d at %lf.", msg->clouds.size(), now);

    std_msgs::Header header = msg->header;
    //header.frame_id = frame_id_;
    //header.stamp = ros::Time::now();

    // initial coarse segments directly from segment node or after classified by learning node
    std::vector<autosense::PointICloudPtr> segment_clouds;
    for (size_t i = 0u; i < msg->clouds.size(); ++i) 
    {
        autosense::PointICloudPtr cloud(new autosense::PointICloud);
        pcl::fromROSMsg(msg->clouds[i], *cloud);
        segment_clouds.push_back(cloud);
    }

    // current pose
    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    auto status = Tracker::getSensorPose(tfBuffer_, header.frame_id, map_frame_id_, header.stamp, &pose);
    // auto status = autosense::common::transform::getVelodynePose(
    //     tfBuffer_, sensor_frame_id_, map_frame_id_, now, &pose);
    if (!status) 
    {
        ROS_WARN("Failed to fetch current pose, tracking skipped...");
        return;
    }
    auto velo2world = std::make_shared<Eigen::Matrix4d>(pose);

    // object builder
    autosense::common::Clock clock_builder;
    std::vector<autosense::ObjectPtr> objects;
    object_builder_->build(segment_clouds, &objects);
    ROS_INFO_STREAM("Objects built. Took " << clock_builder.takeRealTime() << "ms.");

    // visualize initial coarse segments
    autosense::common::publishObjectsMarkers(pub_segments_coarse_, header, autosense::common::MAGENTA.rgbA, objects);

   /**
     * @brief Use Tracking temporal information to improve segmentation
     * @note
     *  <1> project coarse segmentation into World coordinate
     *  <2> get expected objectd (in world coordinate)
     *  <3> over-segmentation/under-segmentation detect and improve segmentation
     *  <4> project back to current Velodyne coordinate
     */
    std::vector<autosense::ObjectPtr> expected_objects =
        tracking_worker_->collectExpectedObjects(now.toSec(), *velo2world);
    std::vector<autosense::ObjectPtr> obsv_objects(objects.begin(), objects.end());
    /// Tracking
    if (!expected_objects.empty()) {
        for (size_t expected_idx = 0u; expected_idx < expected_objects.size();
             ++expected_idx) {
            autosense::common::bbox::GroundBox gbox_expected;
            autosense::common::bbox::toGroundBox(expected_objects[expected_idx], &gbox_expected);

            autosense::ObjectPtr object_merged(new autosense::Object);

            for (size_t obsv_idx = 0u; obsv_idx < objects.size(); ++obsv_idx) {
                autosense::common::bbox::GroundBox gbox_obsv;
                autosense::common::bbox::toGroundBox(objects[obsv_idx], &gbox_obsv);

                // combining all connected components within an expected
                // object’s bounding box into a new one
                if (autosense::common::bbox::groundBoxOverlap( gbox_expected, gbox_obsv, threshold_contian_IoU_)) 
                {
                    *object_merged->cloud += *objects[obsv_idx]->cloud;
                    obsv_objects[obsv_idx]->cloud->clear();
                }
            }
            // build merged object
            object_builder_->build(object_merged);
            // maintain tracking-help segmented objects
            obsv_objects.push_back(object_merged);
        }
        // remove all connected components at once
        auto iter = obsv_objects.begin();
        for (; iter != obsv_objects.end();) 
        {
            if ((*iter)->cloud->empty()) 
            {
                iter = obsv_objects.erase(iter);
            } else 
            {
                ++iter;
            }
        }
    }

    // visualize expected objects
    autosense::common::publishObjectsMarkers(pub_segments_predict_, header, autosense::common::DARKGREEN.rgbA, expected_objects);
    // visualize segmentation results
    autosense::common::publishObjectsMarkers(pub_segments_, header, autosense::common::GREEN.rgbA, obsv_objects);

    autosense::tracking::TrackingOptions tracking_options;
    tracking_options.velo2world_trans = velo2world;
    std::vector<autosense::ObjectPtr> tracking_objects_velo;
    autosense::common::Clock clock_tracking;
    tracking_worker_->track(obsv_objects, now.toSec(), tracking_options, &tracking_objects_velo);
    ROS_INFO_STREAM("Finish tracking. "
                    << tracking_objects_velo.size() << " Objects Tracked. Took "
                    << clock_tracking.takeRealTime() << "ms.");

    /**
     * publish tracking object clouds for classification
     *   object state: ground center & yaw & velocity
     *   object size, observed segment & its id
     */
    const std::vector<autosense::ObjectPtr> &tracking_objects_world =
        tracking_worker_->collectTrackingObjectsInWorld();
    autosense::common::publishTrackingObjects(pub_tracking_output_objects_, header, tracking_objects_world);
    // publish fixed trajectory for classification
    const std::vector<autosense::FixedTrajectory> &fixed_trajectories =
        tracking_worker_->collectFixedTrajectories();
    autosense::common::publishTrackingFixedTrajectories(pub_tracking_output_trajectories_, header, fixed_trajectories);

    // visualize tracking process results, Object Trajectories
    const std::map<autosense::IdType, autosense::Trajectory> &trajectories =
        tracking_worker_->collectTrajectories();
    autosense::common::publishObjectsTrajectory(pub_tracking_objects_trajectory_, header, pose.inverse(), trajectories);
    autosense::common::publishObjectsMarkers(pub_tracking_objects_, header,autosense::common::CYAN.rgbA, tracking_objects_velo);
    // construct tracking-help segmentation results
    std::vector<autosense::PointICloudPtr> objects_cloud;
    for (size_t idx = 0u; idx < tracking_objects_velo.size(); ++idx) 
    {
        objects_cloud.push_back(tracking_objects_velo[idx]->cloud);
    }
    autosense::common::publishClustersCloud<autosense::PointI>(pub_tracking_objects_cloud_, header, objects_cloud);
    // Velocity value and direction
    autosense::common::publishObjectsVelocityArrow(pub_tracking_objects_velocity_, header, autosense::common::RED.rgbA, tracking_objects_velo);
  }
  
  bool Tracker::getSensorPose(const tf2_ros::Buffer& tf_buffer,
                            const std::string& source_frame,
                            const std::string& target_frame,
                            const ros::Time& query_time,
                            Eigen::Matrix4d* trans) 
  {
    if (trans == nullptr) {
        ROS_ERROR("Failed to get trans, the trans ptr can not be NULL.");
        return false;
    }

    geometry_msgs::TransformStamped transform_stamped;
    try 
    {
        transform_stamped = tf_buffer.lookupTransform(target_frame, source_frame, query_time);
    } 
    catch (tf2::TransformException& ex) 
    {
        ROS_WARN(
            "Failed to query pose at %lf, use latest available pose instead.",
            query_time);
        try {
            transform_stamped = tf_buffer.lookupTransform(target_frame, source_frame, ros::Time(0));
        } catch (tf2::TransformException& ex) {
            ROS_ERROR_STREAM("Exception: " << ex.what());
            return false;
        }
    }
    Eigen::Affine3d affine_3d = tf2::transformToEigen(transform_stamped);
    //tf::transformTFToEigen(transform_stamped, affine_3d);
    *trans = affine_3d.matrix();

    ROS_INFO_STREAM("Get " << source_frame << " to " << target_frame
                           << " trans: \n"
                           << *trans);

    return true;
}


} // namespace Cloud_Segmentor

