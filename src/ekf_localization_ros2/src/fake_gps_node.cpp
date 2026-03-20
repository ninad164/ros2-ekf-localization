#include <chrono>
#include <cmath>
#include <memory>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

class FakeGpsNode : public rclcpp::Node
{
public:
  FakeGpsNode()
  : Node("fake_gps_node"),
    rng_(std::random_device{}()),
    noise_x_(0.0, 0.3),
    noise_y_(0.0, 0.3)
  {
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom",
      10,
      std::bind(&FakeGpsNode::odomCallback, this, std::placeholders::_1));

    gps_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
      "/fake_gps",
      10);

    RCLCPP_INFO(this->get_logger(), "Fake GPS node started.");
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    geometry_msgs::msg::PointStamped gps_msg;
    gps_msg.header.stamp = msg->header.stamp;
    gps_msg.header.frame_id = "map";

    gps_msg.point.x = msg->pose.pose.position.x + noise_x_(rng_);
    gps_msg.point.y = msg->pose.pose.position.y + noise_y_(rng_);
    gps_msg.point.z = 0.0;

    gps_pub_->publish(gps_msg);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr gps_pub_;

  std::mt19937 rng_;
  std::normal_distribution<double> noise_x_;
  std::normal_distribution<double> noise_y_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FakeGpsNode>());
  rclcpp::shutdown();
  return 0;
}