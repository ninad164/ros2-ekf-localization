#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "ekf_localization_ros2/ekf.hpp"

class EKFNode : public rclcpp::Node
{
public:
  EKFNode()
  : Node("ekf_node"),
    last_odom_time_(0, 0, RCL_ROS_TIME)
  {
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom",
      10,
      std::bind(&EKFNode::odomCallback, this, std::placeholders::_1));

    ekf_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
      "/ekf/odom",
      10);

    gps_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/fake_gps",
      10,
      std::bind(&EKFNode::gpsCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "EKF node started.");
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const rclcpp::Time current_time = msg->header.stamp;

    const double x = msg->pose.pose.position.x;
    const double y = msg->pose.pose.position.y;

    const double qx = msg->pose.pose.orientation.x;
    const double qy = msg->pose.pose.orientation.y;
    const double qz = msg->pose.pose.orientation.z;
    const double qw = msg->pose.pose.orientation.w;

    const double theta = std::atan2(
      2.0 * (qw * qz + qx * qy),
      1.0 - 2.0 * (qy * qy + qz * qz));

    if (!ekf_.isInitialized()) {
      ekf_.initialize(x, y, theta);
      last_odom_time_ = current_time;
      RCLCPP_INFO(this->get_logger(), "EKF initialized from /odom");
      publishEkfOdometry(current_time);
      return;
    }

    const double dt = (current_time - last_odom_time_).seconds();
    last_odom_time_ = current_time;

    const double v = msg->twist.twist.linear.x;
    const double w = msg->twist.twist.angular.z;

    ekf_.predict(v, w, dt);
    publishEkfOdometry(current_time);
  }

  void publishEkfOdometry(const rclcpp::Time & stamp)
  {
    const Eigen::Vector3d state = ekf_.getState();

    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = stamp;
    odom_msg.header.frame_id = "map";
    odom_msg.child_frame_id = "base_link";

    odom_msg.pose.pose.position.x = state(0);
    odom_msg.pose.pose.position.y = state(1);
    odom_msg.pose.pose.position.z = 0.0;

    const double theta = state(2);
    odom_msg.pose.pose.orientation.x = 0.0;
    odom_msg.pose.pose.orientation.y = 0.0;
    odom_msg.pose.pose.orientation.z = std::sin(theta / 2.0);
    odom_msg.pose.pose.orientation.w = std::cos(theta / 2.0);

    ekf_odom_pub_->publish(odom_msg);
  }

  void gpsCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    if (!ekf_.isInitialized()) {
      return;
    }

    const double meas_x = msg->point.x;
    const double meas_y = msg->point.y;

    ekf_.updatePosition(meas_x, meas_y);
  }
  
  EKF ekf_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr ekf_odom_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr gps_sub_;

  rclcpp::Time last_odom_time_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EKFNode>());
  rclcpp::shutdown();
  return 0;
}