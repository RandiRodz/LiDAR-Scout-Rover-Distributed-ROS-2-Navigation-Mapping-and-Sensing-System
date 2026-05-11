#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

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

}  // namespace

class Nav2ObstacleCloud : public rclcpp::Node
{
public:
  Nav2ObstacleCloud()
  : Node("nav2_obstacle_cloud")
  {
    cloud_topic_ = this->declare_parameter<std::string>("cloud_topic", "/unilidar/cloud");
    imu_topic_ = this->declare_parameter<std::string>("imu_topic", "/unilidar/imu");
    output_topic_ = this->declare_parameter<std::string>("output_topic", "/nav2_obstacles_360");
    marker_topic_ =
      this->declare_parameter<std::string>("marker_topic", "/nav2_obstacle_markers");
    output_frame_id_ = this->declare_parameter<std::string>("output_frame_id", "base_link");
    sensor_offset_x_ = this->declare_parameter<double>("sensor_offset_x", 0.0);
    sensor_offset_y_ = this->declare_parameter<double>("sensor_offset_y", 0.0);
    sensor_offset_z_ = this->declare_parameter<double>("sensor_offset_z", 0.124);

    use_imu_axis_hint_ = this->declare_parameter<bool>("use_imu_axis_hint", false);
    imu_timeout_sec_ = this->declare_parameter<double>("imu_timeout_sec", 0.25);
    publish_markers_ = this->declare_parameter<bool>("publish_markers", true);

    leaf_size_ = this->declare_parameter<double>("leaf_size", 0.08);
    roi_x_min_ = this->declare_parameter<double>("roi_x_min", -3.0);
    roi_x_max_ = this->declare_parameter<double>("roi_x_max", 3.0);
    roi_y_min_ = this->declare_parameter<double>("roi_y_min", -3.0);
    roi_y_max_ = this->declare_parameter<double>("roi_y_max", 3.0);
    roi_z_min_ = this->declare_parameter<double>("roi_z_min", -1.5);
    roi_z_max_ = this->declare_parameter<double>("roi_z_max", 0.35);

    plane_distance_threshold_ =
      this->declare_parameter<double>("plane_distance_threshold", 0.08);
    plane_angle_tolerance_deg_ =
      this->declare_parameter<double>("plane_angle_tolerance_deg", 20.0);
    plane_max_iterations_ = this->declare_parameter<int>("plane_max_iterations", 150);

    cluster_tolerance_ = this->declare_parameter<double>("cluster_tolerance", 0.18);
    min_cluster_size_ = this->declare_parameter<int>("min_cluster_size", 6);
    max_cluster_size_ = this->declare_parameter<int>("max_cluster_size", 5000);
    verbose_diagnostics_ = this->declare_parameter<bool>("verbose_diagnostics", true);
    diagnostics_period_ms_ = this->declare_parameter<int>("diagnostics_period_ms", 1000);

    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic_, rclcpp::SensorDataQoS(),
      std::bind(&Nav2ObstacleCloud::cloudCallback, this, _1));

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, rclcpp::SensorDataQoS(),
      std::bind(&Nav2ObstacleCloud::imuCallback, this, _1));

    output_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, 10);
    marker_pub_ =
      this->create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic_, 10);

    RCLCPP_INFO(
      this->get_logger(),
      "Listening on %s, publishing 360 Nav2 obstacles on %s and markers on %s",
      cloud_topic_.c_str(), output_topic_.c_str(), marker_topic_.c_str());
    RCLCPP_INFO(
      this->get_logger(),
      "360 ROI x:[%.2f, %.2f] y:[%.2f, %.2f] z:[%.2f, %.2f]",
      roi_x_min_, roi_x_max_, roi_y_min_, roi_y_max_, roi_z_min_, roi_z_max_);
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
      publishEmptyObstacleCloud(*msg);
      return;
    }

    auto finite_cloud = std::make_shared<pcl::PointCloud<PointT>>();
    std::vector<int> finite_indices;
    pcl::removeNaNFromPointCloud(*input_cloud, *finite_cloud, finite_indices);
    const std::size_t finite_point_count = finite_cloud->size();

    if (finite_cloud->empty()) {
      publishEmptyObstacleCloud(*msg);
      return;
    }

    const auto roi_cloud = cropCloud(finite_cloud);
    const std::size_t roi_point_count = roi_cloud->size();
    if (roi_cloud->empty()) {
      publishEmptyObstacleCloud(*msg);
      return;
    }

    const auto downsampled_cloud = voxelizeCloud(roi_cloud);
    const std::size_t downsampled_point_count = downsampled_cloud->size();
    if (downsampled_cloud->empty()) {
      publishEmptyObstacleCloud(*msg);
      return;
    }

    const auto ground_result = removeGroundPlane(downsampled_cloud, msg->header.stamp);
    const auto & obstacle_cloud = ground_result.obstacle_cloud;
    const std::size_t obstacle_point_count = obstacle_cloud->size();
    if (obstacle_cloud->empty()) {
      logProcessingSummary(
        raw_point_count, finite_point_count, roi_point_count, downsampled_point_count,
        ground_result.ground_inlier_count, obstacle_point_count, 0U);
      publishEmptyObstacleCloud(*msg);
      return;
    }

    std::vector<ClusterBounds> cluster_bounds_list;
    cluster_bounds_list.reserve(16);

    if (publish_markers_) {
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

      for (const auto & cluster : cluster_indices) {
        ClusterBounds bounds;
        for (const int point_index : cluster.indices) {
          bounds.update(obstacle_cloud->points[point_index]);
        }
        cluster_bounds_list.push_back(bounds);
      }
    }

    publishObstacleCloud(*msg, obstacle_cloud);
    publishMarkers(msg->header, cluster_bounds_list);
    logProcessingSummary(
      raw_point_count, finite_point_count, roi_point_count, downsampled_point_count,
      ground_result.ground_inlier_count, obstacle_point_count, cluster_bounds_list.size());
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
        "Nav2 obstacle ground plane was not found. Proceeding with the cropped cloud.");
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
      return Eigen::Vector3f::UnitZ();
    }

    const rclcpp::Time cloud_time(cloud_stamp);
    if (cloud_time.nanoseconds() > 0 &&
      latest_imu_stamp_.nanoseconds() > 0 &&
      std::abs((cloud_time - latest_imu_stamp_).seconds()) > imu_timeout_sec_)
    {
      return Eigen::Vector3f::UnitZ();
    }

    Eigen::Vector3f axis_hint = latest_imu_orientation_.conjugate() * Eigen::Vector3f::UnitZ();
    if (!axis_hint.allFinite() || axis_hint.norm() < 1e-6f) {
      return Eigen::Vector3f::UnitZ();
    }

    return axis_hint.normalized();
  }

  void publishObstacleCloud(
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
    output_msg.header.frame_id =
      output_frame_id_.empty() ? reference_msg.header.frame_id : output_frame_id_;
    output_pub_->publish(output_msg);
  }

  void publishEmptyObstacleCloud(const sensor_msgs::msg::PointCloud2 & reference_msg)
  {
    auto empty_cloud = std::make_shared<pcl::PointCloud<PointT>>();
    empty_cloud->width = 0;
    empty_cloud->height = 1;
    empty_cloud->is_dense = true;
    publishObstacleCloud(reference_msg, empty_cloud);
    publishMarkers(reference_msg.header, {});
  }

  void publishMarkers(
    const std_msgs::msg::Header & header,
    const std::vector<ClusterBounds> & cluster_bounds_list)
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
    clear_marker.ns = "nav2_obstacles";
    clear_marker.id = 0;
    clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(clear_marker);

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
      box_marker.color.r = 0.1f;
      box_marker.color.g = 0.8f;
      box_marker.color.b = 0.2f;
      box_marker.color.a = 0.22f;
      marker_array.markers.push_back(box_marker);
    }

    marker_pub_->publish(marker_array);
  }

  void logProcessingSummary(
    const std::size_t raw_point_count, const std::size_t finite_point_count,
    const std::size_t roi_point_count, const std::size_t downsampled_point_count,
    const std::size_t ground_inlier_count, const std::size_t obstacle_point_count,
    const std::size_t cluster_count)
  {
    if (!verbose_diagnostics_) {
      return;
    }

    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), diagnostics_period_ms_,
      "nav2 obstacle stats raw=%zu finite=%zu roi=%zu voxel=%zu ground=%zu obstacles=%zu clusters=%zu",
      raw_point_count, finite_point_count, roi_point_count, downsampled_point_count,
      ground_inlier_count, obstacle_point_count, cluster_count);
  }

  std::string cloud_topic_;
  std::string imu_topic_;
  std::string output_topic_;
  std::string marker_topic_;
  std::string output_frame_id_;
  double sensor_offset_x_{0.0};
  double sensor_offset_y_{0.0};
  double sensor_offset_z_{0.124};

  bool use_imu_axis_hint_{false};
  double imu_timeout_sec_{0.25};
  bool publish_markers_{true};
  double leaf_size_{0.08};
  double roi_x_min_{-3.0};
  double roi_x_max_{3.0};
  double roi_y_min_{-3.0};
  double roi_y_max_{3.0};
  double roi_z_min_{-1.5};
  double roi_z_max_{0.35};
  double plane_distance_threshold_{0.08};
  double plane_angle_tolerance_deg_{20.0};
  int plane_max_iterations_{150};
  double cluster_tolerance_{0.18};
  int min_cluster_size_{6};
  int max_cluster_size_{5000};
  bool verbose_diagnostics_{true};
  int diagnostics_period_ms_{1000};

  mutable std::mutex imu_mutex_;
  bool has_imu_orientation_{false};
  Eigen::Quaternionf latest_imu_orientation_{Eigen::Quaternionf::Identity()};
  rclcpp::Time latest_imu_stamp_{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr output_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Nav2ObstacleCloud>());
  rclcpp::shutdown();
  return 0;
}
