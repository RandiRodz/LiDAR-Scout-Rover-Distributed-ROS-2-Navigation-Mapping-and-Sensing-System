#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

using std::placeholders::_1;

class LidarRetimestampBridge : public rclcpp::Node
{
public:
  LidarRetimestampBridge()
  : Node("lidar_retimestamp_bridge")
  {
    input_cloud_topic_ =
      this->declare_parameter<std::string>("input_cloud_topic", "/unilidar/cloud");
    input_imu_topic_ =
      this->declare_parameter<std::string>("input_imu_topic", "/unilidar/imu");
    output_cloud_topic_ =
      this->declare_parameter<std::string>("output_cloud_topic", "/unilidar/cloud_restamped");
    output_imu_topic_ =
      this->declare_parameter<std::string>("output_imu_topic", "/unilidar/imu_restamped");

    // Point-LIO subscribes to IMU data with a reliable QoS profile, so publish the
    // restamped IMU stream reliably while keeping the cloud on the usual sensor-data QoS.
    const auto reliable_imu_qos = rclcpp::QoS(rclcpp::KeepLast(200)).reliable();

    cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      output_cloud_topic_, rclcpp::SensorDataQoS());
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
      output_imu_topic_, reliable_imu_qos);

    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      input_cloud_topic_, rclcpp::SensorDataQoS(),
      std::bind(&LidarRetimestampBridge::cloudCallback, this, _1));

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      input_imu_topic_, rclcpp::SensorDataQoS(),
      std::bind(&LidarRetimestampBridge::imuCallback, this, _1));

    RCLCPP_INFO(
      this->get_logger(),
      "Restamping %s -> %s and %s -> %s",
      input_cloud_topic_.c_str(), output_cloud_topic_.c_str(),
      input_imu_topic_.c_str(), output_imu_topic_.c_str());
  }

private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    auto republished_msg = *msg;
    republished_msg.header.stamp = this->now();
    cloud_pub_->publish(republished_msg);
  }

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    auto republished_msg = *msg;
    republished_msg.header.stamp = this->now();
    imu_pub_->publish(republished_msg);
  }

  std::string input_cloud_topic_;
  std::string input_imu_topic_;
  std::string output_cloud_topic_;
  std::string output_imu_topic_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarRetimestampBridge>());
  rclcpp::shutdown();
  return 0;
}
