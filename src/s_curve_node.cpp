#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

class SCurveNode : public rclcpp::Node
{
public:
  SCurveNode()
  : Node("s_curve_node")
  {
    pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    start_time_ = this->now();
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&SCurveNode::timer_callback, this));
  }

private:
  void timer_callback()
  {
    const rclcpp::Time now = this->now();
    const double t = (now - start_time_).seconds();
    const double duration = 60.0;

    geometry_msgs::msg::Twist twist;
    if (t < duration) {
      twist.linear.x = 0.30;
      twist.angular.z = 0.4 * std::sin(0.2 * t);
    } else {
      twist.linear.x = 0.0;
      twist.angular.z = 0.0;
      pub_->publish(twist);
      RCLCPP_INFO(this->get_logger(), "S-curve finished, stopping turtlebot.");
      rclcpp::shutdown();
      return;
    }

    pub_->publish(twist);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time start_time_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SCurveNode>());
  rclcpp::shutdown();
  return 0;
}
