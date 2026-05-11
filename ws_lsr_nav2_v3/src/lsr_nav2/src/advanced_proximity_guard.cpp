#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/header.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include <Eigen/Geometry>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>

using std::placeholders::_1;

namespace
{

using PointT = pcl::PointXYZ;

constexpr double kPi = 3.14159265358979323846;

struct ClusterBounds
{
  float min_x{std::numeric_limits<float>::infinity()};
  float max_x{-std::numeric_limits<float>::infinity()};
  float min_y{std::numeric_limits<float>::infinity()};
  float max_y{-std::numeric_limits<float>::infinity()};
  float min_z{std::numeric_limits<float>::infinity()};
  float max_z{-std::numeric_limits<float>::infinity()};

  void update(const PointT & point)
  {
    min_x = std::min(min_x, point.x);
    max_x = std::max(max_x, point.x);
    min_y = std::min(min_y, point.y);
    max_y = std::max(max_y, point.y);
    min_z = std::min(min_z, point.z);
    max_z = std::max(max_z, point.z);
  }

  float center_y() const
  {
    return 0.5f * (min_y + max_y);
  }
};

struct GroundSegmentationResult
{
  pcl::PointCloud<PointT>::Ptr obstacle_cloud{new pcl::PointCloud<PointT>()};
  std::size_t ground_inlier_count{0};
  bool plane_found{false};
};

double degToRad(const double degrees)
{
  return degrees * kPi / 180.0;
}

double chooseTurnDirection(const ClusterBounds & bounds)
{
  if (bounds.min_y <= 0.0f && bounds.max_y >= 0.0f) {
    return std::abs(bounds.max_y) >= std::abs(bounds.min_y) ? -1.0 : 1.0;
  }

  return bounds.center_y() >= 0.0f ? -1.0 : 1.0;
}

}  // namespace

class AdvancedProximityGuard : public rclcpp::Node
{
public:
  AdvancedProximityGuard()
  : Node("advanced_proximity_guard")
  {
    cloud_topic_ = this->declare_parameter<std::string>("cloud_topic", "/unilidar/cloud");
    imu_topic_ = this->declare_parameter<std::string>("imu_topic", "/unilidar/imu");
    avoid_topic_ = this->declare_parameter<std::string>("avoid_topic", "cmd_vel_avoid");
    filtered_topic_ = this->declare_parameter<std::string>("filtered_topic", "/filtered_obstacles");
    marker_topic_ = this->declare_parameter<std::string>("marker_topic", "/obstacle_markers");
    output_frame_id_ = this->declare_parameter<std::string>("output_frame_id", "base_link");
    sensor_offset_x_ = this->declare_parameter<double>("sensor_offset_x", 0.0);
    sensor_offset_y_ = this->declare_parameter<double>("sensor_offset_y", 0.0);
    sensor_offset_z_ = this->declare_parameter<double>("sensor_offset_z", 0.124);

    use_imu_axis_hint_ = this->declare_parameter<bool>("use_imu_axis_hint", false);
    imu_timeout_sec_ = this->declare_parameter<double>("imu_timeout_sec", 0.25);
    publish_markers_ = this->declare_parameter<bool>("publish_markers", true);

    leaf_size_ = this->declare_parameter<double>("leaf_size", 0.05);
    roi_x_min_ = this->declare_parameter<double>("roi_x_min", 0.0);
    roi_x_max_ = this->declare_parameter<double>("roi_x_max", 3.0);
    roi_y_min_ = this->declare_parameter<double>("roi_y_min", -2.0);
    roi_y_max_ = this->declare_parameter<double>("roi_y_max", 2.0);
    roi_z_min_ = this->declare_parameter<double>("roi_z_min", -1.5);
    roi_z_max_ = this->declare_parameter<double>("roi_z_max", 1.5);

    plane_distance_threshold_ =
      this->declare_parameter<double>("plane_distance_threshold", 0.08);
    plane_angle_tolerance_deg_ =
      this->declare_parameter<double>("plane_angle_tolerance_deg", 20.0);
    plane_max_iterations_ = this->declare_parameter<int>("plane_max_iterations", 150);

    cluster_tolerance_ = this->declare_parameter<double>("cluster_tolerance", 0.14);
    min_cluster_size_ = this->declare_parameter<int>("min_cluster_size", 4);
    max_cluster_size_ = this->declare_parameter<int>("max_cluster_size", 5000);

    danger_x_min_ = this->declare_parameter<double>("danger_x_min", 0.16);
    danger_x_max_ = this->declare_parameter<double>("danger_x_max", 0.52);
    danger_y_min_ = this->declare_parameter<double>("danger_y_min", -0.18);
    danger_y_max_ = this->declare_parameter<double>("danger_y_max", 0.18);

    avoid_linear_x_ = this->declare_parameter<double>("avoid_linear_x", -0.10);
    avoid_angular_z_ = this->declare_parameter<double>("avoid_angular_z", 0.60);
    verbose_diagnostics_ = this->declare_parameter<bool>("verbose_diagnostics", true);
    diagnostics_period_ms_ = this->declare_parameter<int>("diagnostics_period_ms", 1000);

    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic_, rclcpp::SensorDataQoS(),
      std::bind(&AdvancedProximityGuard::cloudCallback, this, _1));

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, rclcpp::SensorDataQoS(),
      std::bind(&AdvancedProximityGuard::imuCallback, this, _1));

    avoid_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(avoid_topic_, 10);
    filtered_pub_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>(filtered_topic_, 10);
    marker_pub_ =
      this->create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic_, 10);

    RCLCPP_INFO(
      this->get_logger(),
      "Listening on %s, publishing avoidance on %s, filtered obstacles on %s, markers on %s",
      cloud_topic_.c_str(), avoid_topic_.c_str(), filtered_topic_.c_str(), marker_topic_.c_str());
    RCLCPP_INFO(
      this->get_logger(),
      "Danger zone x:[%.2f, %.2f] y:[%.2f, %.2f], ROI x:[%.2f, %.2f] y:[%.2f, %.2f] z:[%.2f, %.2f]",
      danger_x_min_, danger_x_max_, danger_y_min_, danger_y_max_,
      roi_x_min_, roi_x_max_, roi_y_min_, roi_y_max_, roi_z_min_, roi_z_max_);
    if (use_imu_axis_hint_) {
      RCLCPP_INFO(
        this->get_logger(),
        "IMU axis hint enabled. This assumes the IMU orientation is aligned with the point cloud frame.");
    }
  }

private:
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    if (msg->orientation_covariance[0] < 0.0) {
      return;
    }

    const auto & q = msg->orientation;
    const double norm =
      std::sqrt((q.x * q.x) + (q.y * q.y) + (q.z * q.z) + (q.w * q.w));
    if (norm < 1e-6 || !std::isfinite(norm)) {
      return;
    }

    std::lock_guard<std::mutex> lock(imu_mutex_);
    latest_imu_orientation_ =
      Eigen::Quaternionf(static_cast<float>(q.w), static_cast<float>(q.x),
      static_cast<float>(q.y), static_cast<float>(q.z)).normalized();
    latest_imu_stamp_ = rclcpp::Time(msg->header.stamp);
    has_imu_orientation_ = true;
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    auto input_cloud = std::make_shared<pcl::PointCloud<PointT>>();
    pcl::fromROSMsg(*msg, *input_cloud);
    const std::size_t raw_point_count = input_cloud->size();

    if (input_cloud->empty()) {
      publishEmptyFilteredCloud(*msg);
      return;
    }

    auto finite_cloud = std::make_shared<pcl::PointCloud<PointT>>();
    std::vector<int> finite_indices;
    pcl::removeNaNFromPointCloud(*input_cloud, *finite_cloud, finite_indices);
    const std::size_t finite_point_count = finite_cloud->size();

    if (finite_cloud->empty()) {
      publishEmptyFilteredCloud(*msg);
      return;
    }

    const auto roi_cloud = cropCloud(finite_cloud);
    const std::size_t roi_point_count = roi_cloud->size();
    if (roi_cloud->empty()) {
      publishEmptyFilteredCloud(*msg);
      return;
    }

    const auto downsampled_cloud = voxelizeCloud(roi_cloud);
    const std::size_t downsampled_point_count = downsampled_cloud->size();
    if (downsampled_cloud->empty()) {
      publishEmptyFilteredCloud(*msg);
      return;
    }

    const auto ground_result = removeGroundPlane(downsampled_cloud, msg->header.stamp);
    const auto & obstacle_cloud = ground_result.obstacle_cloud;
    const std::size_t obstacle_point_count = obstacle_cloud->size();
    if (obstacle_cloud->empty()) {
      logProcessingSummary(
        raw_point_count, finite_point_count, roi_point_count, downsampled_point_count,
        ground_result.ground_inlier_count, obstacle_point_count, 0, false);
      publishEmptyFilteredCloud(*msg);
      return;
    }

    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
    tree->setInputCloud(obstacle_cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> cluster_extractor;
    cluster_extractor.setClusterTolerance(cluster_tolerance_);
    cluster_extractor.setMinClusterSize(min_cluster_size_);
    cluster_extractor.setMaxClusterSize(max_cluster_size_);
    cluster_extractor.setSearchMethod(tree);
    cluster_extractor.setInputCloud(obstacle_cloud);
    cluster_extractor.extract(cluster_indices);

    bool danger_detected = false;
    float closest_danger_x = std::numeric_limits<float>::infinity();
    ClusterBounds chosen_cluster_bounds;

    std::vector<ClusterBounds> cluster_bounds_list;
    cluster_bounds_list.reserve(cluster_indices.size());
    std::vector<bool> cluster_is_danger;
    cluster_is_danger.reserve(cluster_indices.size());

    for (std::size_t cluster_index = 0; cluster_index < cluster_indices.size(); ++cluster_index) {
      ClusterBounds bounds;

      for (const int point_index : cluster_indices[cluster_index].indices) {
        const auto & point = obstacle_cloud->points[point_index];
        bounds.update(point);
      }

      cluster_bounds_list.push_back(bounds);
      cluster_is_danger.push_back(intersectsDangerZone(bounds));

      if (cluster_is_danger.back() && bounds.min_x < closest_danger_x) {
        danger_detected = true;
        closest_danger_x = bounds.min_x;
        chosen_cluster_bounds = bounds;
      }
    }

    publishFilteredCloud(*msg, obstacle_cloud);
    publishMarkers(msg->header, cluster_bounds_list, cluster_is_danger);
    logProcessingSummary(
      raw_point_count, finite_point_count, roi_point_count, downsampled_point_count,
      ground_result.ground_inlier_count, obstacle_point_count, cluster_indices.size(),
      danger_detected);

    if (!danger_detected) {
      return;
    }

    geometry_msgs::msg::Twist avoidance_cmd;
    const bool obstacle_crosses_centerline =
      chosen_cluster_bounds.min_y <= 0.05f && chosen_cluster_bounds.max_y >= -0.05f;
    const double turn_direction = chooseTurnDirection(chosen_cluster_bounds);
    const double reverse_scale = obstacle_crosses_centerline ? 0.6 : 1.0;
    const double turn_scale = obstacle_crosses_centerline ? 1.25 : 1.0;

    avoidance_cmd.linear.x = avoid_linear_x_ * reverse_scale;
    avoidance_cmd.angular.z = turn_direction * std::abs(avoid_angular_z_) * turn_scale;
    avoid_pub_->publish(avoidance_cmd);

    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), diagnostics_period_ms_,
      "Danger cluster x:[%.2f, %.2f] y:[%.2f, %.2f] z:[%.2f, %.2f], publishing cmd_vel_avoid",
      chosen_cluster_bounds.min_x, chosen_cluster_bounds.max_x,
      chosen_cluster_bounds.min_y, chosen_cluster_bounds.max_y,
      chosen_cluster_bounds.min_z, chosen_cluster_bounds.max_z);
  }

  pcl::PointCloud<PointT>::Ptr cropCloud(
    const pcl::PointCloud<PointT>::Ptr & input_cloud) const
  {
    auto cropped_cloud = applyPassThroughFilter(input_cloud, "x", roi_x_min_, roi_x_max_);
    cropped_cloud = applyPassThroughFilter(cropped_cloud, "y", roi_y_min_, roi_y_max_);
    cropped_cloud = applyPassThroughFilter(cropped_cloud, "z", roi_z_min_, roi_z_max_);
    return cropped_cloud;
  }

  pcl::PointCloud<PointT>::Ptr voxelizeCloud(
    const pcl::PointCloud<PointT>::Ptr & input_cloud) const
  {
    auto downsampled_cloud = std::make_shared<pcl::PointCloud<PointT>>();
    pcl::VoxelGrid<PointT> voxel_filter;
    voxel_filter.setInputCloud(input_cloud);
    voxel_filter.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
    voxel_filter.filter(*downsampled_cloud);
    return downsampled_cloud;
  }

  GroundSegmentationResult removeGroundPlane(
    const pcl::PointCloud<PointT>::Ptr & input_cloud,
    const builtin_interfaces::msg::Time & cloud_stamp)
  {
    GroundSegmentationResult result;

    pcl::SACSegmentation<PointT> plane_segmenter;
    plane_segmenter.setOptimizeCoefficients(true);
    plane_segmenter.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    plane_segmenter.setMethodType(pcl::SAC_RANSAC);
    plane_segmenter.setAxis(groundAxisHint(cloud_stamp));
    plane_segmenter.setEpsAngle(degToRad(plane_angle_tolerance_deg_));
    plane_segmenter.setMaxIterations(plane_max_iterations_);
    plane_segmenter.setDistanceThreshold(plane_distance_threshold_);
    plane_segmenter.setInputCloud(input_cloud);

    pcl::PointIndices::Ptr ground_inliers(new pcl::PointIndices());
    pcl::ModelCoefficients::Ptr ground_coefficients(new pcl::ModelCoefficients());
    plane_segmenter.segment(*ground_inliers, *ground_coefficients);

    if (ground_inliers->indices.empty()) {
      *result.obstacle_cloud = *input_cloud;
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), diagnostics_period_ms_,
        "Ground plane was not found. Proceeding with the cropped cloud as obstacle data.");
      return result;
    }

    result.plane_found = true;
    result.ground_inlier_count = ground_inliers->indices.size();

    pcl::ExtractIndices<PointT> extract_indices;
    extract_indices.setInputCloud(input_cloud);
    extract_indices.setIndices(ground_inliers);
    extract_indices.setNegative(true);
    extract_indices.filter(*result.obstacle_cloud);
    return result;
  }

  pcl::PointCloud<PointT>::Ptr applyPassThroughFilter(
    const pcl::PointCloud<PointT>::Ptr & input_cloud, const std::string & field_name,
    const double min_limit, const double max_limit) const
  {
    auto filtered_cloud = std::make_shared<pcl::PointCloud<PointT>>();
    pcl::PassThrough<PointT> pass_filter;
    pass_filter.setInputCloud(input_cloud);
    pass_filter.setFilterFieldName(field_name);
    pass_filter.setFilterLimits(min_limit, max_limit);
    pass_filter.filter(*filtered_cloud);
    return filtered_cloud;
  }

  Eigen::Vector3f groundAxisHint(const builtin_interfaces::msg::Time & cloud_stamp) const
  {
    if (!use_imu_axis_hint_) {
      return Eigen::Vector3f::UnitZ();
    }

    std::lock_guard<std::mutex> lock(imu_mutex_);
    if (!has_imu_orientation_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "IMU axis hint is enabled, but no valid IMU orientation has been received yet.");
      return Eigen::Vector3f::UnitZ();
    }

    const rclcpp::Time cloud_time(cloud_stamp);
    if (cloud_time.nanoseconds() > 0 &&
      latest_imu_stamp_.nanoseconds() > 0 &&
      std::abs((cloud_time - latest_imu_stamp_).seconds()) > imu_timeout_sec_)
    {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "IMU orientation is stale relative to the point cloud. Falling back to +Z for ground fitting.");
      return Eigen::Vector3f::UnitZ();
    }

    // This assumes the IMU orientation describes the sensor frame used by /unilidar/cloud.
    Eigen::Vector3f axis_hint = latest_imu_orientation_.conjugate() * Eigen::Vector3f::UnitZ();
    if (!axis_hint.allFinite() || axis_hint.norm() < 1e-6f) {
      return Eigen::Vector3f::UnitZ();
    }

    return axis_hint.normalized();
  }

  bool intersectsDangerZone(const ClusterBounds & bounds) const
  {
    const bool overlaps_x = bounds.max_x >= danger_x_min_ && bounds.min_x <= danger_x_max_;
    const float center_y = bounds.center_y();
    const bool center_in_lane = center_y >= danger_y_min_ && center_y <= danger_y_max_;
    return overlaps_x && center_in_lane;
  }

  void publishFilteredCloud(
    const sensor_msgs::msg::PointCloud2 & reference_msg,
    const pcl::PointCloud<PointT>::Ptr & obstacle_cloud)
  {
    auto output_cloud = std::make_shared<pcl::PointCloud<PointT>>(*obstacle_cloud);
    if (output_frame_id_ == "base_link") {
      for (auto & point : output_cloud->points) {
        point.x += static_cast<float>(sensor_offset_x_);
        point.y += static_cast<float>(sensor_offset_y_);
        point.z += static_cast<float>(sensor_offset_z_);
      }
    }

    sensor_msgs::msg::PointCloud2 output_msg;
    pcl::toROSMsg(*output_cloud, output_msg);
    output_msg.header.stamp = this->now();
    output_msg.header.frame_id = output_frame_id_.empty() ? reference_msg.header.frame_id : output_frame_id_;
    filtered_pub_->publish(output_msg);
  }

  void publishEmptyFilteredCloud(const sensor_msgs::msg::PointCloud2 & reference_msg)
  {
    auto empty_cloud = std::make_shared<pcl::PointCloud<PointT>>();
    empty_cloud->width = 0;
    empty_cloud->height = 1;
    empty_cloud->is_dense = true;
    publishFilteredCloud(reference_msg, empty_cloud);
    publishMarkers(reference_msg.header, {}, {});
  }

  void publishMarkers(
    const std_msgs::msg::Header & header,
    const std::vector<ClusterBounds> & cluster_bounds_list,
    const std::vector<bool> & cluster_is_danger)
  {
    if (!publish_markers_) {
      return;
    }

    visualization_msgs::msg::MarkerArray marker_array;
    const auto marker_stamp = this->now();

    visualization_msgs::msg::Marker clear_marker;
    clear_marker.header = header;
    clear_marker.header.stamp = marker_stamp;
    clear_marker.header.frame_id = output_frame_id_.empty() ? header.frame_id : output_frame_id_;
    clear_marker.ns = "obstacle_guard";
    clear_marker.id = 0;
    clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(clear_marker);

    visualization_msgs::msg::Marker zone_marker;
    zone_marker.header = clear_marker.header;
    zone_marker.ns = "danger_zone";
    zone_marker.id = 1;
    zone_marker.type = visualization_msgs::msg::Marker::CUBE;
    zone_marker.action = visualization_msgs::msg::Marker::ADD;
    zone_marker.pose.position.x = sensor_offset_x_ + 0.5 * (danger_x_min_ + danger_x_max_);
    zone_marker.pose.position.y = sensor_offset_y_ + 0.5 * (danger_y_min_ + danger_y_max_);
    zone_marker.pose.position.z = sensor_offset_z_ + 0.01;
    zone_marker.pose.orientation.w = 1.0;
    zone_marker.scale.x = std::max(0.02, danger_x_max_ - danger_x_min_);
    zone_marker.scale.y = std::max(0.02, danger_y_max_ - danger_y_min_);
    zone_marker.scale.z = 0.02;
    zone_marker.color.r = 1.0f;
    zone_marker.color.g = 0.1f;
    zone_marker.color.b = 0.1f;
    zone_marker.color.a = 0.18f;
    marker_array.markers.push_back(zone_marker);

    for (std::size_t index = 0; index < cluster_bounds_list.size(); ++index) {
      const auto & bounds = cluster_bounds_list[index];
      visualization_msgs::msg::Marker box_marker;
      box_marker.header = clear_marker.header;
      box_marker.ns = "clusters";
      box_marker.id = static_cast<int>(10 + index);
      box_marker.type = visualization_msgs::msg::Marker::CUBE;
      box_marker.action = visualization_msgs::msg::Marker::ADD;
      box_marker.pose.position.x = sensor_offset_x_ + 0.5 * (bounds.min_x + bounds.max_x);
      box_marker.pose.position.y = sensor_offset_y_ + 0.5 * (bounds.min_y + bounds.max_y);
      box_marker.pose.position.z = sensor_offset_z_ + 0.5 * (bounds.min_z + bounds.max_z);
      box_marker.pose.orientation.w = 1.0;
      box_marker.scale.x = std::max(0.02f, bounds.max_x - bounds.min_x);
      box_marker.scale.y = std::max(0.02f, bounds.max_y - bounds.min_y);
      box_marker.scale.z = std::max(0.02f, bounds.max_z - bounds.min_z);
      box_marker.color.r = cluster_is_danger[index] ? 1.0f : 0.0f;
      box_marker.color.g = cluster_is_danger[index] ? 0.2f : 0.8f;
      box_marker.color.b = cluster_is_danger[index] ? 0.2f : 1.0f;
      box_marker.color.a = cluster_is_danger[index] ? 0.40f : 0.25f;
      marker_array.markers.push_back(box_marker);
    }

    marker_pub_->publish(marker_array);
  }

  void logProcessingSummary(
    const std::size_t raw_point_count, const std::size_t finite_point_count,
    const std::size_t roi_point_count, const std::size_t downsampled_point_count,
    const std::size_t ground_inlier_count, const std::size_t obstacle_point_count,
    const std::size_t cluster_count, const bool danger_detected)
  {
    if (!verbose_diagnostics_) {
      return;
    }

    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), diagnostics_period_ms_,
      "cloud stats raw=%zu finite=%zu roi=%zu voxel=%zu ground=%zu obstacles=%zu clusters=%zu danger=%s",
      raw_point_count, finite_point_count, roi_point_count, downsampled_point_count,
      ground_inlier_count, obstacle_point_count, cluster_count,
      danger_detected ? "true" : "false");
  }

  std::string cloud_topic_;
  std::string imu_topic_;
  std::string avoid_topic_;
  std::string filtered_topic_;
  std::string marker_topic_;
  std::string output_frame_id_;
  double sensor_offset_x_{0.0};
  double sensor_offset_y_{0.0};
  double sensor_offset_z_{0.124};

  bool use_imu_axis_hint_{false};
  double imu_timeout_sec_{0.25};
  bool publish_markers_{true};
  double leaf_size_{0.08};
  double roi_x_min_{0.0};
  double roi_x_max_{3.0};
  double roi_y_min_{-2.0};
  double roi_y_max_{2.0};
  double roi_z_min_{-1.5};
  double roi_z_max_{1.5};
  double plane_distance_threshold_{0.08};
  double plane_angle_tolerance_deg_{20.0};
  int plane_max_iterations_{150};
  double cluster_tolerance_{0.18};
  int min_cluster_size_{20};
  int max_cluster_size_{5000};
  double danger_x_min_{0.10};
  double danger_x_max_{0.60};
  double danger_y_min_{-0.30};
  double danger_y_max_{0.30};
  double avoid_linear_x_{-0.10};
  double avoid_angular_z_{0.60};
  bool verbose_diagnostics_{true};
  int diagnostics_period_ms_{2000};

  mutable std::mutex imu_mutex_;
  bool has_imu_orientation_{false};
  Eigen::Quaternionf latest_imu_orientation_{Eigen::Quaternionf::Identity()};
  rclcpp::Time latest_imu_stamp_{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr avoid_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AdvancedProximityGuard>());
  rclcpp::shutdown();
  return 0;
}
