#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class KFNode : public rclcpp::Node
{
public:
  KFNode() : Node("kf_node")
  {
    sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&KFNode::odom_callback, this, std::placeholders::_1));

    pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/kf/pose", 10);
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    geometry_msgs::msg::PoseStamped out;
    pub_->publish(out);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KFNode>());
    rclcpp::shutdown();
    return 0;
}