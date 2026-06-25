#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <array>
#include <memory>
#include <cmath>

class KFNode : public rclcpp::Node
{
public:
  KFNode() : Node("kf_node")
  {
    sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&KFNode::odom_callback, this, std::placeholders::_1));

    pub_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/kf/pose", 10);
    pub_path_ = this->create_publisher<nav_msgs::msg::Path>("/kf/path", 10);
    pub_cov_ = this->create_publisher<visualization_msgs::msg::Marker>("/kf/covariance", 10);
    pub_arrow_ = this->create_publisher<visualization_msgs::msg::Marker>("/kf/pose_arrow", 10);

    path_.header.frame_id = "map";

    P_.fill(0.0);
    Q_.fill(0.0);
    R_.fill(0.0);
    P_[0] = 0.20;
    P_[1 * 4 + 1] = 0.08;
    P_[2 * 4 + 2] = 0.05;
    P_[3 * 4 + 3] = 0.05;
    Q_[0] = 0.01;
    Q_[1 * 4 + 1] = 0.02;
    Q_[2 * 4 + 2] = 0.01;
    Q_[3 * 4 + 3] = 0.01;
    R_[0] = 0.05;
    R_[1 * 4 + 1] = 0.08;
    R_[2 * 4 + 2] = 0.05;
    R_[3 * 4 + 3] = 0.05;
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const rclcpp::Time stamp(msg->header.stamp);
    if (!initialized_) {
      state_[0] = msg->pose.pose.position.x;
      state_[1] = msg->pose.pose.position.y;
      state_[2] = msg->twist.twist.linear.x;
      state_[3] = msg->twist.twist.linear.y;
      last_time_ = stamp;
      initialized_ = true;
      publish_state(stamp);
      return;
    }

    const double dt = std::max(0.001, (stamp - last_time_).seconds());
    last_time_ = stamp;

    const std::array<double, 16> A = {
      1.0, 0.0, dt,  0.0,
      0.0, 1.0, 0.0, dt,
      0.0, 0.0, 1.0, 0.0,
      0.0, 0.0, 0.0, 1.0
    };

    const std::array<double, 4> z = {
      msg->pose.pose.position.x,
      msg->pose.pose.position.y,
      msg->twist.twist.linear.x,
      msg->twist.twist.linear.y
    };

    const std::array<double, 4> x_pred = mat4_vec_mul(A, state_);
    const std::array<double, 16> P_pred = mat4_add(mat4_mul(mat4_mul(A, P_), mat4_transpose(A)), Q_);

    const std::array<double, 16> S = mat4_add(P_pred, R_);
    const std::array<double, 16> S_inv = invert4(S);
    const std::array<double, 16> K = mat4_mul(P_pred, S_inv);

    std::array<double, 4> y;
    for (int i = 0; i < 4; ++i) {
      y[i] = z[i] - x_pred[i];
    }

    const std::array<double, 4> K_y = mat4_vec_mul(K, y);
    for (int i = 0; i < 4; ++i) {
      state_[i] = x_pred[i] + K_y[i];
    }

    const std::array<double, 16> I = identity4();
    const std::array<double, 16> temp = mat4_sub(I, K);
    P_ = mat4_mul(temp, P_pred);

    publish_state(stamp);
  }

  void publish_state(const rclcpp::Time &stamp)
  {
    geometry_msgs::msg::PoseStamped out;
    out.header.stamp = stamp;
    out.header.frame_id = "map";
    out.pose.position.x = state_[0];
    out.pose.position.y = state_[1];
    out.pose.position.z = 0.0;
    const double yaw = std::atan2(state_[3], state_[2]);
    out.pose.orientation = quaternion_from_yaw(yaw);
    pub_pose_->publish(out);

    path_.header.stamp = stamp;
    path_.poses.push_back(out);
    pub_path_->publish(path_);

    pub_cov_->publish(make_covariance_marker(out, P_));
    pub_arrow_->publish(make_arrow_marker(out));
  }

  visualization_msgs::msg::Marker make_covariance_marker(
      const geometry_msgs::msg::PoseStamped &pose,
      const std::array<double, 16> &P) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header = pose.header;
    marker.ns = "kf_covariance";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.06;
    marker.color.r = 0.0;
    marker.color.g = 0.65;
    marker.color.b = 1.0;
    marker.color.a = 0.9;

    const double a = P[0];
    const double b = P[1];
    const double c = P[5];
    const double trace = a + c;
    const double det = a * c - b * b;
    const double lambda = std::max(0.0, 0.5 * (trace + std::sqrt(std::max(0.0, trace * trace - 4.0 * det))));
    const double lambda2 = std::max(0.0, 0.5 * (trace - std::sqrt(std::max(0.0, trace * trace - 4.0 * det))));
    const double angle = 0.5 * std::atan2(2.0 * b, a - c);
    const double r1 = 3.0 * std::sqrt(lambda);
    const double r2 = 3.0 * std::sqrt(lambda2);

    for (int i = 0; i <= 32; ++i) {
      const double theta = 2.0 * M_PI * i / 32.0;
      const double x = r1 * std::cos(theta);
      const double y = r2 * std::sin(theta);
      const double gx = std::cos(angle) * x - std::sin(angle) * y + pose.pose.position.x;
      const double gy = std::sin(angle) * x + std::cos(angle) * y + pose.pose.position.y;
      geometry_msgs::msg::Point p;
      p.x = gx;
      p.y = gy;
      p.z = 0.0;
      marker.points.push_back(p);
    }

    return marker;
  }

  visualization_msgs::msg::Marker make_arrow_marker(const geometry_msgs::msg::PoseStamped &pose) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header = pose.header;
    marker.ns = "kf_pose_arrow";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose = pose.pose;
    marker.scale.x = 0.6;
    marker.scale.y = 0.12;
    marker.scale.z = 0.12;
    marker.color.r = 0.9;
    marker.color.g = 0.2;
    marker.color.b = 0.2;
    marker.color.a = 0.9;
    return marker;
  }

  static std::array<double, 4> mat4_vec_mul(const std::array<double, 16> &m, const std::array<double, 4> &v)
  {
    std::array<double, 4> out{};
    for (int i = 0; i < 4; ++i) {
      out[i] = m[i * 4 + 0] * v[0] + m[i * 4 + 1] * v[1] + m[i * 4 + 2] * v[2] + m[i * 4 + 3] * v[3];
    }
    return out;
  }

  static std::array<double, 16> mat4_mul(const std::array<double, 16> &a, const std::array<double, 16> &b)
  {
    std::array<double, 16> out{};
    for (int row = 0; row < 4; ++row) {
      for (int col = 0; col < 4; ++col) {
        double sum = 0.0;
        for (int k = 0; k < 4; ++k) {
          sum += a[row * 4 + k] * b[k * 4 + col];
        }
        out[row * 4 + col] = sum;
      }
    }
    return out;
  }

  static std::array<double, 16> mat4_add(const std::array<double, 16> &a, const std::array<double, 16> &b)
  {
    std::array<double, 16> out;
    for (int i = 0; i < 16; ++i) {
      out[i] = a[i] + b[i];
    }
    return out;
  }

  static std::array<double, 16> mat4_sub(const std::array<double, 16> &a, const std::array<double, 16> &b)
  {
    std::array<double, 16> out;
    for (int i = 0; i < 16; ++i) {
      out[i] = a[i] - b[i];
    }
    return out;
  }

  static std::array<double, 16> mat4_transpose(const std::array<double, 16> &m)
  {
    std::array<double, 16> out;
    for (int row = 0; row < 4; ++row) {
      for (int col = 0; col < 4; ++col) {
        out[row * 4 + col] = m[col * 4 + row];
      }
    }
    return out;
  }

  static std::array<double, 16> identity4()
  {
    std::array<double, 16> out{};
    for (int i = 0; i < 4; ++i) {
      out[i * 4 + i] = 1.0;
    }
    return out;
  }

  static geometry_msgs::msg::Quaternion quaternion_from_yaw(double yaw)
  {
    geometry_msgs::msg::Quaternion q;
    q.w = std::cos(yaw * 0.5);
    q.x = 0.0;
    q.y = 0.0;
    q.z = std::sin(yaw * 0.5);
    return q;
  }

  static std::array<double, 16> invert4(std::array<double, 16> m)
  {
    std::array<double, 16> inv;
    std::array<double, 32> aug{};
    for (int row = 0; row < 4; ++row) {
      for (int col = 0; col < 4; ++col) {
        aug[row * 8 + col] = m[row * 4 + col];
        aug[row * 8 + 4 + col] = (row == col) ? 1.0 : 0.0;
      }
    }
    for (int i = 0; i < 4; ++i) {
      double pivot = aug[i * 8 + i];
      if (std::fabs(pivot) < 1e-12) pivot = 1e-12;
      for (int col = 0; col < 8; ++col) {
        aug[i * 8 + col] /= pivot;
      }
      for (int row = 0; row < 4; ++row) {
        if (row == i) continue;
        const double factor = aug[row * 8 + i];
        for (int col = 0; col < 8; ++col) {
          aug[row * 8 + col] -= factor * aug[i * 8 + col];
        }
      }
    }
    for (int row = 0; row < 4; ++row) {
      for (int col = 0; col < 4; ++col) {
        inv[row * 4 + col] = aug[row * 8 + 4 + col];
      }
    }
    return inv;
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_cov_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_arrow_;
  nav_msgs::msg::Path path_;
  std::array<double, 4> state_{};
  std::array<double, 16> P_;
  std::array<double, 16> Q_;
  std::array<double, 16> R_;
  bool initialized_ = false;
  rclcpp::Time last_time_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KFNode>());
  rclcpp::shutdown();
  return 0;
}
