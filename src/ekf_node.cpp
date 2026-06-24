#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <memory>

class EKFNode : public rclcpp::Node
{
public:
  EKFNode() : Node("ekf_node")
  {
    sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&EKFNode::odom_callback, this, std::placeholders::_1));

    pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/ekf/pose", 10);
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    geometry_msgs::msg::PoseStamped out;
    out.header = msg->header;
    out.pose = msg->pose.pose;
    pub_->publish(out);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EKFNode>());
  rclcpp::shutdown();
  return 0;
}
