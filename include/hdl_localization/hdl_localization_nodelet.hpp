#include <mutex>
#include <memory>
#include <iostream>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <eigen_conversions/eigen_msg.h>

#include <std_srvs/Empty.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <pcl/filters/voxel_grid.h>

#include <pclomp/ndt_omp.h>
#include <fast_gicp/ndt/ndt_cuda.hpp>

#include <hdl_localization/pose_estimator.hpp>
#include <hdl_localization/delta_estimater.hpp>
#include <string>

#include <hdl_localization/ScanMatchingStatus.h>
#include <hdl_global_localization/SetGlobalMap.h>
#include <hdl_global_localization/QueryGlobalLocalization.h>

namespace hdl_localization {
using PointT = pcl::PointXYZI;

class HdlLocalizationNodelet : public nodelet::Nodelet {
public:
  HdlLocalizationNodelet() : tf_buffer(), tf_listener(tf_buffer), relocalizing(false) {};
  virtual ~HdlLocalizationNodelet() override {};
  void onInit() override;

private:
  pcl::Registration<PointT, PointT>::Ptr create_registration() const;
  pcl::PointCloud<PointT>::ConstPtr downsample(const pcl::PointCloud<PointT>::ConstPtr& cloud) const;
  void initialize_params();
  void imu_callback(const sensor_msgs::Imu::ConstPtr& imu_msg);
  void points_callback(const sensor_msgs::PointCloud2::ConstPtr& points_msg);
  void globalmap_callback(const sensor_msgs::PointCloud2::ConstPtr& points_msg);
  bool relocalize(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& res);
  void initialpose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_msg);
  void publish_odometry(const ros::Time& stamp, const Eigen::Matrix4f& pose);
  void publish_transform(const ros::Time& stamp, const Eigen::Matrix4f& pose);
  void publish_scan_matching_status(const std_msgs::Header& header, pcl::PointCloud<pcl::PointXYZI>::ConstPtr aligned);

private:
  // ROS
  ros::NodeHandle nh;
  ros::NodeHandle mt_nh;
  ros::NodeHandle private_nh;

  std::string robot_odom_frame_id;
  std::string odom_child_frame_id;
  std::string reg_method;
  std::string ndt_neighbor_search_method;
  std::string points_topic;
  std::string imu_topic;

  bool use_imu;
  bool invert_acc;
  bool invert_gyro;
  bool enable_robot_odometry_prediction;
  bool enable_aligned_cloud_publish;
  bool enable_scan_matching_status_publish;
  bool enable_odometry_publish;
  bool enable_transform_publish;

  double downsample_resolution;
  double ndt_neighbor_search_radius;
  double ndt_resolution;

  ros::Subscriber imu_sub;
  ros::Subscriber points_sub;
  ros::Subscriber globalmap_sub;
  ros::Subscriber initialpose_sub;

  ros::Publisher pose_pub;
  ros::Publisher aligned_pub;
  ros::Publisher status_pub;

  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener;
  tf2_ros::TransformBroadcaster tf_broadcaster;

  // imu input buffer
  std::mutex imu_data_mutex;
  std::vector<sensor_msgs::ImuConstPtr> imu_data;

  // globalmap and registration method
  pcl::PointCloud<PointT>::Ptr globalmap;
  pcl::Filter<PointT>::Ptr downsample_filter;
  pcl::Registration<PointT, PointT>::Ptr registration;

  // pose estimator
  std::mutex pose_estimator_mutex;
  std::unique_ptr<hdl_localization::PoseEstimator> pose_estimator;

  // global localization
  bool use_global_localization;
  std::atomic_bool relocalizing;
  std::unique_ptr<DeltaEstimater> delta_estimater;

  pcl::PointCloud<PointT>::ConstPtr last_scan;
  ros::ServiceServer relocalize_server;
  ros::ServiceClient set_global_map_service;
  ros::ServiceClient query_global_localization_service;
};
}  // namespace hdl_localization
