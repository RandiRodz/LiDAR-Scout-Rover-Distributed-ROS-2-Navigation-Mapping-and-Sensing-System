#include <algorithm>
#include <array>
#include <chrono>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;

class CmdVelMux : public rclcpp::Node
{
public:
  CmdVelMux()
  : Node("cmd_vel_mux")
  {
    output_topic_ = this->declare_parameter<std::string>("output_topic", "/cmd_vel");
    nav_topic_ = this->declare_parameter<std::string>("nav_topic", "cmd_vel_nav");
    teleop_topic_ = this->declare_parameter<std::string>("teleop_topic", "cmd_vel_teleop");
    avoid_topic_ = this->declare_parameter<std::string>("avoid_topic", "cmd_vel_avoid");
    nav_timeout_sec_ = this->declare_parameter<double>("nav_timeout_sec", 0.5);
    teleop_timeout_sec_ = this->declare_parameter<double>("teleop_timeout_sec", 0.5);
    avoid_timeout_sec_ = this->declare_parameter<double>("avoid_timeout_sec", 0.25);
    publish_rate_hz_ = this->declare_parameter<double>("publish_rate_hz", 20.0);

    output_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(output_topic_, 10);

    nav_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      nav_topic_, 10, std::bind(&CmdVelMux::navCallback, this, _1));
    teleop_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      teleop_topic_, 10, std::bind(&CmdVelMux::teleopCallback, this, _1));
    avoid_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      avoid_topic_, 10, std::bind(&CmdVelMux::avoidCallback, this, _1));

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate_hz_));
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&CmdVelMux::publishSelectedCommand, this));

    RCLCPP_INFO(
      this->get_logger(),
      "Muxing %s, %s, %s -> %s",
      avoid_topic_.c_str(), teleop_topic_.c_str(), nav_topic_.c_str(), output_topic_.c_str());
  }

private:
  enum class Source
  {
    None,
    Avoid,
    Teleop,
    Nav
  };

  struct CachedCommand
  {
    geometry_msgs::msg::Twist msg{};
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    bool received{false};
  };

  void navCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    nav_cmd_.msg = *msg;
    nav_cmd_.stamp = this->now();
    nav_cmd_.received = true;
  }

  void teleopCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    teleop_cmd_.msg = *msg;
    teleop_cmd_.stamp = this->now();
    teleop_cmd_.received = true;
  }

  void avoidCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    avoid_cmd_.msg = *msg;
    avoid_cmd_.stamp = this->now();
    avoid_cmd_.received = true;
  }

  bool isFresh(const CachedCommand & cmd, const double timeout_sec) const
  {
    if (!cmd.received) {
      return false;
    }

    return (this->now() - cmd.stamp).seconds() <= timeout_sec;
  }

  void publishSelectedCommand()
  {
    const auto selected = selectSource();

    geometry_msgs::msg::Twist output_msg;
    if (selected == Source::Avoid) {
      output_msg = avoid_cmd_.msg;
    } else if (selected == Source::Teleop) {
      output_msg = teleop_cmd_.msg;
    } else if (selected == Source::Nav) {
      output_msg = nav_cmd_.msg;
    } else if (!published_stop_) {
      output_pub_->publish(output_msg);
      published_stop_ = true;
      last_source_ = Source::None;
      return;
    } else {
      return;
    }

    output_pub_->publish(output_msg);
    published_stop_ = false;

    if (selected != last_source_) {
      RCLCPP_INFO(
        this->get_logger(),
        "Active cmd_vel source: %s",
        selected == Source::Avoid ? "avoidance" :
        selected == Source::Teleop ? "teleop" :
        selected == Source::Nav ? "navigation" : "none");
      last_source_ = selected;
    }
  }

  Source selectSource() const
  {
    if (isFresh(avoid_cmd_, avoid_timeout_sec_)) {
      return Source::Avoid;
    }
    if (isFresh(teleop_cmd_, teleop_timeout_sec_)) {
      return Source::Teleop;
    }
    if (isFresh(nav_cmd_, nav_timeout_sec_)) {
      return Source::Nav;
    }
    return Source::None;
  }

  std::string output_topic_;
  std::string nav_topic_;
  std::string teleop_topic_;
  std::string avoid_topic_;
  double nav_timeout_sec_{0.5};
  double teleop_timeout_sec_{0.5};
  double avoid_timeout_sec_{0.25};
  double publish_rate_hz_{20.0};
  bool published_stop_{false};
  Source last_source_{Source::None};

  CachedCommand nav_cmd_;
  CachedCommand teleop_cmd_;
  CachedCommand avoid_cmd_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr teleop_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr avoid_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr output_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdVelMux>());
  rclcpp::shutdown();
  return 0;
}
