#include "hdl_localization/hdl_localization_nodelet.hpp"

namespace hdl_localization {

void HdlLocalizationNodelet::onInit() {
  nh = getNodeHandle();
  mt_nh = getMTNodeHandle();
  private_nh = getPrivateNodeHandle();

  initialize_params();

  if (use_imu) {
    NODELET_INFO("enable imu-based prediction");
    imu_sub = mt_nh.subscribe(imu_topic, 256, &HdlLocalizationNodelet::imu_callback, this);
  }
  points_sub = mt_nh.subscribe(points_topic, 5, &HdlLocalizationNodelet::points_callback, this);
  globalmap_sub = nh.subscribe("/globalmap", 1, &HdlLocalizationNodelet::globalmap_callback, this);
  initialpose_sub = nh.subscribe("/initialpose", 8, &HdlLocalizationNodelet::initialpose_callback, this);

  pose_pub = nh.advertise<nav_msgs::Odometry>("/odom", 5, false);
  aligned_pub = nh.advertise<sensor_msgs::PointCloud2>("/aligned_points", 5, false);
  status_pub = nh.advertise<ScanMatchingStatus>("/status", 5, false);

  // global localization
  if (use_global_localization) {
    NODELET_INFO_STREAM("wait for global localization services");
    ros::service::waitForService("/hdl_global_localization/set_global_map");
    ros::service::waitForService("/hdl_global_localization/query");

    set_global_map_service = nh.serviceClient<hdl_global_localization::SetGlobalMap>("/hdl_global_localization/set_global_map");
    query_global_localization_service = nh.serviceClient<hdl_global_localization::QueryGlobalLocalization>("/hdl_global_localization/query");

    relocalize_server = nh.advertiseService("/relocalize", &HdlLocalizationNodelet::relocalize, this);
  }
}

pcl::Registration<PointT, PointT>::Ptr HdlLocalizationNodelet::create_registration() const {
  if (reg_method == "NDT_OMP") {
    NODELET_INFO("NDT_OMP is selected");
    pclomp::NormalDistributionsTransform<PointT, PointT>::Ptr ndt(new pclomp::NormalDistributionsTransform<PointT, PointT>());
    ndt->setTransformationEpsilon(0.01);
    ndt->setResolution(ndt_resolution);
    if (ndt_neighbor_search_method == "DIRECT1") {
      NODELET_INFO("search_method DIRECT1 is selected");
      ndt->setNeighborhoodSearchMethod(pclomp::DIRECT1);
    } else if (ndt_neighbor_search_method == "DIRECT7") {
      NODELET_INFO("search_method DIRECT7 is selected");
      ndt->setNeighborhoodSearchMethod(pclomp::DIRECT7);
    } else {
      if (ndt_neighbor_search_method == "KDTREE") {
        NODELET_INFO("search_method KDTREE is selected");
      } else {
        NODELET_WARN("invalid search method was given");
        NODELET_WARN("default method is selected (KDTREE)");
      }
      ndt->setNeighborhoodSearchMethod(pclomp::KDTREE);
    }
    return ndt;
  } else if (reg_method.find("NDT_CUDA") != std::string::npos) {
    NODELET_INFO("NDT_CUDA is selected");
    boost::shared_ptr<fast_gicp::NDTCuda<PointT, PointT>> ndt(new fast_gicp::NDTCuda<PointT, PointT>);
    ndt->setResolution(ndt_resolution);

    if (reg_method.find("D2D") != std::string::npos) {
      ndt->setDistanceMode(fast_gicp::NDTDistanceMode::D2D);
    } else if (reg_method.find("P2D") != std::string::npos) {
      ndt->setDistanceMode(fast_gicp::NDTDistanceMode::P2D);
    }

    if (ndt_neighbor_search_method == "DIRECT1") {
      NODELET_INFO("search_method DIRECT1 is selected");
      ndt->setNeighborSearchMethod(fast_gicp::NeighborSearchMethod::DIRECT1);
    } else if (ndt_neighbor_search_method == "DIRECT7") {
      NODELET_INFO("search_method DIRECT7 is selected");
      ndt->setNeighborSearchMethod(fast_gicp::NeighborSearchMethod::DIRECT7);
    } else if (ndt_neighbor_search_method == "DIRECT_RADIUS") {
      NODELET_INFO_STREAM("search_method DIRECT_RADIUS is selected : " << ndt_neighbor_search_radius);
      ndt->setNeighborSearchMethod(fast_gicp::NeighborSearchMethod::DIRECT_RADIUS, ndt_neighbor_search_radius);
    } else {
      NODELET_WARN("invalid search method was given");
    }
    return ndt;
  }

  NODELET_ERROR_STREAM("unknown registration method:" << reg_method);
  return nullptr;
}

void HdlLocalizationNodelet::initialize_params() {
  use_imu = private_nh.param<bool>("use_imu", true);
  invert_acc = private_nh.param<bool>("invert_acc", false);
  invert_gyro = private_nh.param<bool>("invert_gyro", false);
  use_global_localization = private_nh.param<bool>("use_global_localization", true);
  enable_robot_odometry_prediction = private_nh.param<bool>("enable_robot_odometry_prediction", true);
  enable_aligned_cloud_publish = private_nh.param<bool>("enable_aligned_cloud_publish", false);
  enable_scan_matching_status_publish = private_nh.param<bool>("enable_scan_matching_status_publish", false);
  enable_odometry_publish = private_nh.param<bool>("enable_odometry_publish", true);
  enable_transform_publish = private_nh.param<bool>("enable_transform_publish", true);

  downsample_resolution = private_nh.param<double>("downsample_resolution", 0.1);
  ndt_resolution = private_nh.param<double>("ndt_resolution", 1.0);
  ndt_neighbor_search_radius = private_nh.param<double>("ndt_neighbor_search_radius", 2.0);

  reg_method = private_nh.param<std::string>("reg_method", "NDT_OMP");
  ndt_neighbor_search_method = private_nh.param<std::string>("ndt_neighbor_search_method", "DIRECT7");
  robot_odom_frame_id = private_nh.param<std::string>("robot_odom_frame_id", "robot_odom");
  odom_child_frame_id = private_nh.param<std::string>("odom_child_frame_id", "base_link");
  points_topic = private_nh.param<std::string>("points_topic", "/robosense/points");
  imu_topic = private_nh.param<std::string>("imu_topic", "/robosense/imu");

  boost::shared_ptr<pcl::VoxelGrid<PointT>> voxelgrid(new pcl::VoxelGrid<PointT>());
  voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
  downsample_filter = voxelgrid;
  NODELET_INFO("create registration method for localization");
  registration = create_registration();

  // global localization
  NODELET_INFO("create registration method for fallback during relocalization");
  relocalizing = false;
  delta_estimater.reset(new DeltaEstimater(create_registration()));

  // initialize pose estimator
  if (private_nh.param<bool>("specify_init_pose", true)) {
    NODELET_INFO("initialize pose estimator with specified parameters!!");
    pose_estimator.reset(new hdl_localization::PoseEstimator(
      registration,
      Eigen::Vector3f(private_nh.param<double>("init_pos_x", 0.0), private_nh.param<double>("init_pos_y", 0.0), private_nh.param<double>("init_pos_z", 0.0)),
      Eigen::Quaternionf(
        private_nh.param<double>("init_ori_w", 1.0),
        private_nh.param<double>("init_ori_x", 0.0),
        private_nh.param<double>("init_ori_y", 0.0),
        private_nh.param<double>("init_ori_z", 0.0)),
      private_nh.param<double>("cool_time_duration", 0.5)));
  }
}

void HdlLocalizationNodelet::imu_callback(const sensor_msgs::Imu::ConstPtr& imu_msg) {
  std::lock_guard<std::mutex> lock(imu_data_mutex);
  imu_data.push_back(imu_msg);
}

void HdlLocalizationNodelet::points_callback(const sensor_msgs::PointCloud2::ConstPtr& points_msg) {
  if (!globalmap) {
    NODELET_ERROR("globalmap has not been received!!");
    return;
  }

  const auto& stamp = points_msg->header.stamp;
  pcl::PointCloud<PointT>::Ptr pcl_cloud(new pcl::PointCloud<PointT>());
  pcl::fromROSMsg(*points_msg, *pcl_cloud);

  if (pcl_cloud->empty()) {
    NODELET_ERROR("cloud is empty!!");
    return;
  }

  // transform pointcloud into odom_child_frame_id (base_link)
  std::string tfError;
  pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
  if (this->tf_buffer.canTransform(odom_child_frame_id, pcl_cloud->header.frame_id, stamp, ros::Duration(0.1), &tfError)) {
    if (!pcl_ros::transformPointCloud(odom_child_frame_id, *pcl_cloud, *cloud, this->tf_buffer)) {
      NODELET_ERROR("point cloud cannot be transformed into target frame!!");
      return;
    }
  } else {
    NODELET_ERROR(tfError.c_str());
    return;
  }

  auto filtered = downsample(cloud);
  last_scan = filtered;

  if (relocalizing) {
    delta_estimater->add_frame(filtered);
  }

  std::lock_guard<std::mutex> estimator_lock(pose_estimator_mutex);
  if (!pose_estimator) {
    NODELET_ERROR("waiting for initial pose input!!");
    return;
  }
  Eigen::Matrix4f before = pose_estimator->matrix();

  // predict
  if (!use_imu) {
    pose_estimator->predict(stamp);
  } else {
    std::lock_guard<std::mutex> lock(imu_data_mutex);
    auto imu_iter = imu_data.begin();
    for (imu_iter; imu_iter != imu_data.end(); imu_iter++) {
      if (stamp < (*imu_iter)->header.stamp) {
        break;
      }
      const auto& acc = (*imu_iter)->linear_acceleration;
      const auto& gyro = (*imu_iter)->angular_velocity;
      double acc_sign = invert_acc ? -1.0 : 1.0;
      double gyro_sign = invert_gyro ? -1.0 : 1.0;
      pose_estimator->predict((*imu_iter)->header.stamp, acc_sign * Eigen::Vector3f(acc.x, acc.y, acc.z), gyro_sign * Eigen::Vector3f(gyro.x, gyro.y, gyro.z));
    }
    imu_data.erase(imu_data.begin(), imu_iter);
  }

  // odometry-based prediction
  ros::Time last_correction_time = pose_estimator->last_correction_time();
  if (enable_robot_odometry_prediction && !last_correction_time.isZero()) {
    geometry_msgs::TransformStamped odom_delta;
    if (tf_buffer.canTransform(odom_child_frame_id, last_correction_time, odom_child_frame_id, stamp, robot_odom_frame_id, ros::Duration(0.1))) {
      odom_delta = tf_buffer.lookupTransform(odom_child_frame_id, last_correction_time, odom_child_frame_id, stamp, robot_odom_frame_id, ros::Duration(0));
    } else if (tf_buffer.canTransform(odom_child_frame_id, last_correction_time, odom_child_frame_id, ros::Time(0), robot_odom_frame_id, ros::Duration(0))) {
      odom_delta = tf_buffer.lookupTransform(odom_child_frame_id, last_correction_time, odom_child_frame_id, ros::Time(0), robot_odom_frame_id, ros::Duration(0));
    }

    if (odom_delta.header.stamp.isZero()) {
      NODELET_WARN_STREAM("failed to look up transform between " << cloud->header.frame_id << " and " << robot_odom_frame_id);
    } else {
      Eigen::Isometry3d delta = tf2::transformToEigen(odom_delta);
      pose_estimator->predict_odom(delta.cast<float>().matrix());
    }
  }

  // align cloud_in_base to globalmap
  auto aligned = pose_estimator->correct(stamp, filtered);

  if (enable_aligned_cloud_publish) {
    aligned->header.frame_id = "map";
    aligned->header.stamp = cloud->header.stamp;
    aligned_pub.publish(aligned);
  }

  if (enable_scan_matching_status_publish) {
    publish_scan_matching_status(points_msg->header, aligned);
  }

  if (enable_odometry_publish) {
    publish_odometry(points_msg->header.stamp, pose_estimator->matrix());
  }

  if (enable_transform_publish) {
    publish_transform(points_msg->header.stamp, pose_estimator->matrix());
  }
}

void HdlLocalizationNodelet::globalmap_callback(const sensor_msgs::PointCloud2::ConstPtr& points_msg) {
  NODELET_INFO("globalmap received!");
  pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
  pcl::fromROSMsg(*points_msg, *cloud);
  globalmap = cloud;

  registration->setInputTarget(globalmap);

  if (use_global_localization) {
    NODELET_INFO("set globalmap for global localization!");
    hdl_global_localization::SetGlobalMap srv;
    pcl::toROSMsg(*globalmap, srv.request.global_map);

    if (!set_global_map_service.call(srv)) {
      NODELET_INFO("failed to set global map");
    } else {
      NODELET_INFO("done");
    }
  }
}

bool HdlLocalizationNodelet::relocalize(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& res) {
  if (last_scan == nullptr) {
    NODELET_INFO_STREAM("no scan has been received");
    return false;
  }

  relocalizing = true;
  delta_estimater->reset();
  pcl::PointCloud<PointT>::ConstPtr scan = last_scan;

  hdl_global_localization::QueryGlobalLocalization srv;
  pcl::toROSMsg(*scan, srv.request.cloud);
  srv.request.max_num_candidates = 1;

  if (!query_global_localization_service.call(srv) || srv.response.poses.empty()) {
    relocalizing = false;
    NODELET_INFO_STREAM("global localization failed");
    return false;
  }

  const auto& result = srv.response.poses[0];

  NODELET_INFO_STREAM("--- Global localization result ---");
  NODELET_INFO_STREAM("Trans :" << result.position.x << " " << result.position.y << " " << result.position.z);
  NODELET_INFO_STREAM("Quat  :" << result.orientation.x << " " << result.orientation.y << " " << result.orientation.z << " " << result.orientation.w);
  NODELET_INFO_STREAM("Error :" << srv.response.errors[0]);
  NODELET_INFO_STREAM("Inlier:" << srv.response.inlier_fractions[0]);

  Eigen::Isometry3f pose = Eigen::Isometry3f::Identity();
  pose.linear() = Eigen::Quaternionf(result.orientation.w, result.orientation.x, result.orientation.y, result.orientation.z).toRotationMatrix();
  pose.translation() = Eigen::Vector3f(result.position.x, result.position.y, result.position.z);
  pose = pose * delta_estimater->estimated_delta();

  std::lock_guard<std::mutex> lock(pose_estimator_mutex);
  pose_estimator.reset(
    new hdl_localization::PoseEstimator(registration, pose.translation(), Eigen::Quaternionf(pose.linear()), private_nh.param<double>("cool_time_duration", 0.5)));

  relocalizing = false;

  return true;
}

void HdlLocalizationNodelet::initialpose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_msg) {
  NODELET_INFO("initial pose received!!");
  std::lock_guard<std::mutex> lock(pose_estimator_mutex);
  const auto& p = pose_msg->pose.pose.position;
  const auto& q = pose_msg->pose.pose.orientation;
  pose_estimator.reset(
    new hdl_localization::PoseEstimator(registration, Eigen::Vector3f(p.x, p.y, p.z), Eigen::Quaternionf(q.w, q.x, q.y, q.z), private_nh.param<double>("cool_time_duration", 0.5)));
}

pcl::PointCloud<PointT>::ConstPtr HdlLocalizationNodelet::downsample(const pcl::PointCloud<PointT>::ConstPtr& cloud) const {
  if (!downsample_filter) {
    return cloud;
  }

  pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
  downsample_filter->setInputCloud(cloud);
  downsample_filter->filter(*filtered);
  filtered->header = cloud->header;

  return filtered;
}

void HdlLocalizationNodelet::publish_odometry(const ros::Time& stamp, const Eigen::Matrix4f& pose) {
  nav_msgs::Odometry odom;
  odom.header.stamp = stamp;
  odom.header.frame_id = "map";

  tf::poseEigenToMsg(Eigen::Isometry3d(pose.cast<double>()), odom.pose.pose);
  odom.child_frame_id = odom_child_frame_id;
  odom.twist.twist.linear.x = 0.0;
  odom.twist.twist.linear.y = 0.0;
  odom.twist.twist.angular.z = 0.0;

  pose_pub.publish(odom);
}

void HdlLocalizationNodelet::publish_transform(const ros::Time& stamp, const Eigen::Matrix4f& pose) {
  // T_pose^-1 = T_map<-baselink
  if (tf_buffer.canTransform(robot_odom_frame_id, odom_child_frame_id, ros::Time(0))) {
    // T_map<-baselink
    geometry_msgs::TransformStamped map_wrt_frame = tf2::eigenToTransform(Eigen::Isometry3d(pose.inverse().cast<double>()));
    map_wrt_frame.header.stamp = stamp;
    map_wrt_frame.header.frame_id = odom_child_frame_id;
    map_wrt_frame.child_frame_id = "map";

    // T_baselink<-odom
    geometry_msgs::TransformStamped frame_wrt_odom = tf_buffer.lookupTransform(robot_odom_frame_id, odom_child_frame_id, ros::Time(0), ros::Duration(0.1));
    Eigen::Matrix4f frame2odom = tf2::transformToEigen(frame_wrt_odom).cast<float>().matrix();

    // T_map<-odom = T_map<-baselink * T_baselink<-odom
    geometry_msgs::TransformStamped map_wrt_odom;
    tf2::doTransform(map_wrt_frame, map_wrt_odom, frame_wrt_odom);

    // T_odom<-map = (T_map<-odom)^-1
    tf2::Transform odom_wrt_map;
    tf2::fromMsg(map_wrt_odom.transform, odom_wrt_map);
    odom_wrt_map = odom_wrt_map.inverse();

    // publish T_odom<-map
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.transform = tf2::toMsg(odom_wrt_map);
    odom_trans.header.stamp = stamp;
    odom_trans.header.frame_id = "map";
    odom_trans.child_frame_id = robot_odom_frame_id;

    tf_broadcaster.sendTransform(odom_trans);
  } else {
    // publish T_pose = T_baselink<-map
    geometry_msgs::TransformStamped odom_trans = tf2::eigenToTransform(Eigen::Isometry3d(pose.cast<double>()));
    odom_trans.header.stamp = stamp;
    odom_trans.header.frame_id = "map";
    odom_trans.child_frame_id = odom_child_frame_id;
    tf_broadcaster.sendTransform(odom_trans);
  }
}

void HdlLocalizationNodelet::publish_scan_matching_status(const std_msgs::Header& header, pcl::PointCloud<pcl::PointXYZI>::ConstPtr aligned) {
  ScanMatchingStatus status;
  status.header = header;

  status.has_converged = registration->hasConverged();
  status.matching_error = 0.0;

  const double max_correspondence_dist = private_nh.param<double>("status_max_correspondence_dist", 0.5);
  const double max_valid_point_dist = private_nh.param<double>("status_max_valid_point_dist", 25.0);

  int num_inliers = 0;
  int num_valid_points = 0;
  std::vector<int> k_indices;
  std::vector<float> k_sq_dists;
  for (int i = 0; i < aligned->size(); i++) {
    const auto& pt = aligned->at(i);
    if (pt.getVector3fMap().norm() > max_valid_point_dist) {
      continue;
    }
    num_valid_points++;

    registration->getSearchMethodTarget()->nearestKSearch(pt, 1, k_indices, k_sq_dists);
    if (k_sq_dists[0] < max_correspondence_dist * max_correspondence_dist) {
      status.matching_error += k_sq_dists[0];
      num_inliers++;
    }
  }

  status.matching_error /= num_inliers;
  status.inlier_fraction = static_cast<float>(num_inliers) / std::max(1, num_valid_points);
  status.relative_pose = tf2::eigenToTransform(Eigen::Isometry3d(registration->getFinalTransformation().cast<double>())).transform;

  status.prediction_labels.reserve(2);
  status.prediction_errors.reserve(2);

  std::vector<double> errors(6, 0.0);

  if (pose_estimator->wo_prediction_error()) {
    status.prediction_labels.push_back(std_msgs::String());
    status.prediction_labels.back().data = "without_pred";
    status.prediction_errors.push_back(tf2::eigenToTransform(Eigen::Isometry3d(pose_estimator->wo_prediction_error().get().cast<double>())).transform);
  }

  if (pose_estimator->imu_prediction_error()) {
    status.prediction_labels.push_back(std_msgs::String());
    status.prediction_labels.back().data = use_imu ? "imu" : "motion_model";
    status.prediction_errors.push_back(tf2::eigenToTransform(Eigen::Isometry3d(pose_estimator->imu_prediction_error().get().cast<double>())).transform);
  }

  if (pose_estimator->odom_prediction_error()) {
    status.prediction_labels.push_back(std_msgs::String());
    status.prediction_labels.back().data = "odom";
    status.prediction_errors.push_back(tf2::eigenToTransform(Eigen::Isometry3d(pose_estimator->odom_prediction_error().get().cast<double>())).transform);
  }

  status_pub.publish(status);
}

}  // namespace hdl_localization

PLUGINLIB_EXPORT_CLASS(hdl_localization::HdlLocalizationNodelet, nodelet::Nodelet)
