#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

namespace
{

constexpr double kPi = 3.14159265358979323846;

double normalizeAngle(double angle)
{
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
{
  const double siny_cosp = 2.0 * ((q.w * q.z) + (q.x * q.y));
  const double cosy_cosp = 1.0 - (2.0 * ((q.y * q.y) + (q.z * q.z)));
  return std::atan2(siny_cosp, cosy_cosp);
}

geometry_msgs::msg::Quaternion quaternionFromYaw(const double yaw)
{
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  q.normalize();

  geometry_msgs::msg::Quaternion msg;
  msg.x = q.x();
  msg.y = q.y();
  msg.z = q.z();
  msg.w = q.w();
  return msg;
}

double interpolateAngle(const double current, const double target, const double alpha)
{
  return normalizeAngle(current + (alpha * normalizeAngle(target - current)));
}

double clampMagnitude(const double value, const double limit)
{
  return std::clamp(value, -limit, limit);
}

}  // namespace

class OdomStabilizer : public rclcpp::Node
{
public:
  OdomStabilizer()
  : Node("odom_stabilizer")
  {
    input_odom_topic_ = this->declare_parameter<std::string>("input_odom_topic", "/odom_raw");
    output_odom_topic_ = this->declare_parameter<std::string>("output_odom_topic", "/odom");
    cmd_vel_topic_ = this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    odom_frame_ = this->declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");

    alpha_moving_ = this->declare_parameter<double>("alpha_moving", 0.45);
    alpha_stationary_ = this->declare_parameter<double>("alpha_stationary", 0.08);
    moving_linear_threshold_ = this->declare_parameter<double>("moving_linear_threshold", 0.08);
    moving_angular_threshold_ = this->declare_parameter<double>("moving_angular_threshold", 0.20);
    hold_translation_tolerance_ =
      this->declare_parameter<double>("hold_translation_tolerance", 0.12);
    hold_yaw_tolerance_ = this->declare_parameter<double>("hold_yaw_tolerance", 0.10);
    max_translation_rate_moving_ =
      this->declare_parameter<double>("max_translation_rate_moving", 0.80);
    max_translation_rate_stationary_ =
      this->declare_parameter<double>("max_translation_rate_stationary", 0.12);
    max_yaw_rate_moving_ = this->declare_parameter<double>("max_yaw_rate_moving", 1.40);
    max_yaw_rate_stationary_ = this->declare_parameter<double>("max_yaw_rate_stationary", 0.25);
    max_translation_margin_ = this->declare_parameter<double>("max_translation_margin", 0.02);
    max_yaw_margin_ = this->declare_parameter<double>("max_yaw_margin", 0.03);
    reject_translation_jump_moving_ =
      this->declare_parameter<double>("reject_translation_jump_moving", 0.35);
    reject_translation_jump_stationary_ =
      this->declare_parameter<double>("reject_translation_jump_stationary", 0.14);
    reject_yaw_jump_moving_ = this->declare_parameter<double>("reject_yaw_jump_moving", 0.75);
    reject_yaw_jump_stationary_ =
      this->declare_parameter<double>("reject_yaw_jump_stationary", 0.25);
    flatten_z_ = this->declare_parameter<bool>("flatten_z", true);
    publish_tf_ = this->declare_parameter<bool>("publish_tf", true);

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(output_odom_topic_, 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      input_odom_topic_, 20, std::bind(&OdomStabilizer::odomCallback, this, std::placeholders::_1));

    cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic_, 20, std::bind(&OdomStabilizer::cmdCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      this->get_logger(),
      "Stabilizing %s -> %s, TF %s -> %s, cmd topic %s",
      input_odom_topic_.c_str(), output_odom_topic_.c_str(),
      odom_frame_.c_str(), base_frame_.c_str(), cmd_vel_topic_.c_str());
  }

private:
  void cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    last_cmd_linear_ = msg->linear.x;
    last_cmd_angular_ = msg->angular.z;
    last_cmd_stamp_ = this->get_clock()->now();
    has_cmd_ = true;
  }

  bool isCommandedMoving() const
  {
    if (!has_cmd_) {
      return false;
    }

    const auto age = (this->get_clock()->now() - last_cmd_stamp_).seconds();
    if (age > 0.75) {
      return false;
    }

    return std::abs(last_cmd_linear_) > moving_linear_threshold_ ||
           std::abs(last_cmd_angular_) > moving_angular_threshold_;
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const double raw_x = msg->pose.pose.position.x;
    const double raw_y = msg->pose.pose.position.y;
    const double raw_z = flatten_z_ ? 0.0 : msg->pose.pose.position.z;
    const double raw_yaw = yawFromQuaternion(msg->pose.pose.orientation);
    const bool commanded_moving = isCommandedMoving();
    const rclcpp::Time current_stamp =
      (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0) ?
      this->get_clock()->now() : rclcpp::Time(msg->header.stamp);

    if (!has_filtered_pose_) {
      filtered_x_ = raw_x;
      filtered_y_ = raw_y;
      filtered_z_ = raw_z;
      filtered_yaw_ = raw_yaw;
      has_filtered_pose_ = true;
    } else {
      double dt = 0.1;
      if (has_last_odom_stamp_) {
        dt = std::clamp((current_stamp - last_odom_stamp_).seconds(), 0.02, 0.50);
      }

      const double max_translation_rate =
        commanded_moving ? max_translation_rate_moving_ : max_translation_rate_stationary_;
      const double max_yaw_rate =
        commanded_moving ? max_yaw_rate_moving_ : max_yaw_rate_stationary_;
      const double max_translation_step = (max_translation_rate * dt) + max_translation_margin_;
      const double max_yaw_step = (max_yaw_rate * dt) + max_yaw_margin_;
      const double reject_translation_jump = std::max(
        commanded_moving ? reject_translation_jump_moving_ : reject_translation_jump_stationary_,
        4.0 * max_translation_step);
      const double reject_yaw_jump = std::max(
        commanded_moving ? reject_yaw_jump_moving_ : reject_yaw_jump_stationary_,
        4.0 * max_yaw_step);

      double candidate_x = raw_x;
      double candidate_y = raw_y;
      double candidate_z = raw_z;
      double candidate_yaw = raw_yaw;

      const double raw_dx = raw_x - filtered_x_;
      const double raw_dy = raw_y - filtered_y_;
      const double raw_dist = std::hypot(raw_dx, raw_dy);
      const double raw_yaw_delta = normalizeAngle(raw_yaw - filtered_yaw_);
      if (raw_dist > reject_translation_jump || std::abs(raw_yaw_delta) > reject_yaw_jump) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "Rejecting odom outlier: translation %.3f m (limit %.3f), yaw %.3f rad (limit %.3f), moving=%s",
          raw_dist, reject_translation_jump, std::abs(raw_yaw_delta), reject_yaw_jump,
          commanded_moving ? "true" : "false");
      } else {
        if (raw_dist > max_translation_step && raw_dist > 1e-6) {
          const double scale = max_translation_step / raw_dist;
          candidate_x = filtered_x_ + (raw_dx * scale);
          candidate_y = filtered_y_ + (raw_dy * scale);
          candidate_z = flatten_z_ ? 0.0 : (filtered_z_ + ((raw_z - filtered_z_) * scale));
          RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "Clamping odom translation jump from %.3f m to %.3f m (moving=%s)",
            raw_dist, max_translation_step, commanded_moving ? "true" : "false");
        }

        if (std::abs(raw_yaw_delta) > max_yaw_step) {
          candidate_yaw =
            normalizeAngle(filtered_yaw_ + clampMagnitude(raw_yaw_delta, max_yaw_step));
          RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "Clamping odom yaw jump from %.3f rad to %.3f rad (moving=%s)",
            std::abs(raw_yaw_delta), max_yaw_step, commanded_moving ? "true" : "false");
        }

        const double dx = candidate_x - filtered_x_;
        const double dy = candidate_y - filtered_y_;
        const double dist = std::hypot(dx, dy);
        const double yaw_error = std::abs(normalizeAngle(candidate_yaw - filtered_yaw_));

        if (!( !commanded_moving &&
               dist <= hold_translation_tolerance_ &&
               yaw_error <= hold_yaw_tolerance_))
        {
          const double alpha = commanded_moving ? alpha_moving_ : alpha_stationary_;
          filtered_x_ += alpha * dx;
          filtered_y_ += alpha * dy;
          filtered_z_ = flatten_z_ ? 0.0 : (filtered_z_ + (alpha * (candidate_z - filtered_z_)));
          filtered_yaw_ = interpolateAngle(filtered_yaw_, candidate_yaw, alpha);
        }
      }
    }

    last_odom_stamp_ = current_stamp;
    has_last_odom_stamp_ = true;

    nav_msgs::msg::Odometry output = *msg;
    output.header.frame_id = odom_frame_;
    output.child_frame_id = base_frame_;
    output.pose.pose.position.x = filtered_x_;
    output.pose.pose.position.y = filtered_y_;
    output.pose.pose.position.z = flatten_z_ ? 0.0 : filtered_z_;
    output.pose.pose.orientation = quaternionFromYaw(filtered_yaw_);
    odom_pub_->publish(output);

    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped transform;
      transform.header = output.header;
      transform.child_frame_id = base_frame_;
      transform.transform.translation.x = output.pose.pose.position.x;
      transform.transform.translation.y = output.pose.pose.position.y;
      transform.transform.translation.z = output.pose.pose.position.z;
      transform.transform.rotation = output.pose.pose.orientation;
      tf_broadcaster_->sendTransform(transform);
    }
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::string input_odom_topic_;
  std::string output_odom_topic_;
  std::string cmd_vel_topic_;
  std::string odom_frame_;
  std::string base_frame_;

  double alpha_moving_{0.45};
  double alpha_stationary_{0.08};
  double moving_linear_threshold_{0.08};
  double moving_angular_threshold_{0.20};
  double hold_translation_tolerance_{0.12};
  double hold_yaw_tolerance_{0.10};
  double max_translation_rate_moving_{0.80};
  double max_translation_rate_stationary_{0.12};
  double max_yaw_rate_moving_{1.40};
  double max_yaw_rate_stationary_{0.25};
  double max_translation_margin_{0.02};
  double max_yaw_margin_{0.03};
  double reject_translation_jump_moving_{0.35};
  double reject_translation_jump_stationary_{0.14};
  double reject_yaw_jump_moving_{0.75};
  double reject_yaw_jump_stationary_{0.25};
  bool flatten_z_{true};
  bool publish_tf_{true};

  bool has_filtered_pose_{false};
  bool has_last_odom_stamp_{false};
  bool has_cmd_{false};
  double filtered_x_{0.0};
  double filtered_y_{0.0};
  double filtered_z_{0.0};
  double filtered_yaw_{0.0};
  double last_cmd_linear_{0.0};
  double last_cmd_angular_{0.0};
  rclcpp::Time last_cmd_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_odom_stamp_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomStabilizer>());
  rclcpp::shutdown();
  return 0;
}
