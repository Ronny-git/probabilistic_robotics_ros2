#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <memory>

class PFNode : public rclcpp::Node
{
public:
  PFNode() : Node("pf_node")
  {
    sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&PFNode::odom_callback, this, std::placeholders::_1));

    pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/pf/pose", 10);
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
  rclcpp::spin(std::make_shared<PFNode>());
  rclcpp::shutdown();
  return 0;
}
