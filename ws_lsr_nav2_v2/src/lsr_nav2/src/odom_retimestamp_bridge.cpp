#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tf2_ros/transform_broadcaster.h"

using std::placeholders::_1;

class OdomRetimestampBridge : public rclcpp::Node
{
public:
  OdomRetimestampBridge()
  : Node("odom_retimestamp_bridge")
  {
    input_odom_topic_ =
      this->declare_parameter<std::string>("input_odom_topic", "/odom_pointlio_raw");
    input_tf_topic_ =
      this->declare_parameter<std::string>("input_tf_topic", "/tf_pointlio_raw");
    output_odom_topic_ =
      this->declare_parameter<std::string>("output_odom_topic", "/odom");
    odom_frame_ = this->declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");
    input_odom_frame_ =
      this->declare_parameter<std::string>("input_odom_frame", "pointlio_odom_raw");
    input_base_frame_ =
      this->declare_parameter<std::string>("input_base_frame", "pointlio_base_link_raw");
    publish_tf_ = this->declare_parameter<bool>("publish_tf", true);

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(output_odom_topic_, 20);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      input_odom_topic_, 20, std::bind(&OdomRetimestampBridge::odomCallback, this, _1));

    if (!input_tf_topic_.empty()) {
      tf_sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
        input_tf_topic_, 50, std::bind(&OdomRetimestampBridge::tfCallback, this, _1));
    }

    if (input_tf_topic_.empty()) {
      RCLCPP_INFO(
        this->get_logger(),
        "Restamping odom %s (TF input disabled) -> %s -> %s",
        input_odom_topic_.c_str(), odom_frame_.c_str(), base_frame_.c_str());
    } else {
      RCLCPP_INFO(
        this->get_logger(),
        "Restamping odom %s and TF %s (%s -> %s) -> %s -> %s",
        input_odom_topic_.c_str(), input_tf_topic_.c_str(),
        input_odom_frame_.c_str(), input_base_frame_.c_str(),
        odom_frame_.c_str(), base_frame_.c_str());
    }
  }

private:
  void publishTransformAndOdom(
    const geometry_msgs::msg::Transform & source_transform,
    const geometry_msgs::msg::Twist * source_twist = nullptr)
  {
    const auto stamp = this->now();

    nav_msgs::msg::Odometry output;
    output.header.stamp = stamp;
    output.header.frame_id = odom_frame_;
    output.child_frame_id = base_frame_;
    output.pose.pose.position.x = source_transform.translation.x;
    output.pose.pose.position.y = source_transform.translation.y;
    output.pose.pose.position.z = source_transform.translation.z;
    output.pose.pose.orientation = source_transform.rotation;
    if (source_twist != nullptr) {
      output.twist.twist = *source_twist;
    }
    odom_pub_->publish(output);

    if (!publish_tf_) {
      return;
    }

    geometry_msgs::msg::TransformStamped transform;
    transform.header = output.header;
    transform.child_frame_id = output.child_frame_id;
    transform.transform = source_transform;
    tf_broadcaster_->sendTransform(transform);
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    geometry_msgs::msg::Transform source_transform;
    source_transform.translation.x = msg->pose.pose.position.x;
    source_transform.translation.y = msg->pose.pose.position.y;
    source_transform.translation.z = msg->pose.pose.position.z;
    source_transform.rotation = msg->pose.pose.orientation;
    publishTransformAndOdom(source_transform, &msg->twist.twist);
  }

  void tfCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
  {
    for (const auto & transform : msg->transforms) {
      if (transform.header.frame_id != input_odom_frame_ ||
        transform.child_frame_id != input_base_frame_)
      {
        continue;
      }

      publishTransformAndOdom(transform.transform);
      return;
    }
  }

  std::string input_odom_topic_;
  std::string input_tf_topic_;
  std::string output_odom_topic_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string input_odom_frame_;
  std::string input_base_frame_;
  bool publish_tf_{true};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomRetimestampBridge>());
  rclcpp::shutdown();
  return 0;
}
