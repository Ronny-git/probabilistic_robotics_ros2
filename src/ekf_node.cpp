#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <array>
#include <memory>
#include <cmath>

class EKFNode : public rclcpp::Node
{
public:
  EKFNode() : Node("ekf_node")
  {
    sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&EKFNode::odom_callback, this, std::placeholders::_1));

    pub_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/ekf/pose", 10);
    pub_path_ = this->create_publisher<nav_msgs::msg::Path>("/ekf/path", 10);
    pub_cov_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "/ekf/covariance", 10);
    pub_arrow_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "/ekf/pose_arrow", 10);

    path_.header.frame_id = "map";
    P_.fill(0.0);
    Q_.fill(0.0);
    R_.fill(0.0);
    P_[0] = 0.20;
    P_[1 * 4 + 1] = 0.08;
    P_[2 * 4 + 2] = 0.08;
    P_[3 * 4 + 3] = 0.06;
    Q_[0] = 0.02;
    Q_[1 * 4 + 1] = 0.03;
    Q_[2 * 4 + 2] = 0.02;
    Q_[3 * 4 + 3] = 0.02;
    R_[0] = 0.05;
    R_[1 * 4 + 1] = 0.1;
    R_[2 * 4 + 2] = 0.03;
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const rclcpp::Time stamp(msg->header.stamp);
    const double meas_x = msg->pose.pose.position.x;
    const double meas_y = msg->pose.pose.position.y;
    const double meas_theta = yaw_from_quaternion(msg->pose.pose.orientation);
    const double lin_v = std::hypot(msg->twist.twist.linear.x, msg->twist.twist.linear.y);
    const double ang_v = msg->twist.twist.angular.z;

    if (!initialized_) {
      state_[0] = meas_x;
      state_[1] = meas_y;
      state_[2] = meas_theta;
      state_[3] = lin_v;
      last_time_ = stamp;
      initialized_ = true;
      publish_state(stamp);
      return;
    }

    const double dt = std::max(0.001, (stamp - last_time_).seconds());
    last_time_ = stamp;

    const double theta = state_[2];
    const double v = state_[3];
    const double omega = ang_v;

    std::array<double, 4> x_pred = state_;
    x_pred[0] += v * std::cos(theta) * dt;
    x_pred[1] += v * std::sin(theta) * dt;
    x_pred[2] = normalize_angle(theta + omega * dt);
    x_pred[3] = v;

    const std::array<double, 16> F = {
      1.0, 0.0, -v * std::sin(theta) * dt, std::cos(theta) * dt,
      0.0, 1.0,  v * std::cos(theta) * dt, std::sin(theta) * dt,
      0.0, 0.0, 1.0, 0.0,
      0.0, 0.0, 0.0, 1.0
    };

    const std::array<double, 16> P_pred = mat4_add(mat4_mul(mat4_mul(F, P_), mat4_transpose(F)), Q_);
    const std::array<double, 3> z = {meas_x, meas_y, meas_theta};
    const std::array<double, 3> h = {x_pred[0], x_pred[1], x_pred[2]};
    std::array<double, 3> y;
    for (int i = 0; i < 3; ++i) {
      y[i] = z[i] - h[i];
    }
    y[2] = normalize_angle(y[2]);

    const std::array<double, 9> P_pred3 = extract_top_left_3x3(P_pred);
    std::array<double, 9> S = mat3_add(P_pred3, R_);
    const std::array<double, 9> S_inv = invert3(S);
    const std::array<double, 12> P_pred_cols = extract_first_three_columns(P_pred);
    const std::array<double, 12> K = mat4x3_mul_3x3(P_pred_cols, S_inv);

    const std::array<double, 4> K_y = mat4x3_vec_mul(K, y);
    for (int i = 0; i < 4; ++i) {
      state_[i] = x_pred[i] + K_y[i];
    }
    state_[2] = normalize_angle(state_[2]);

    const std::array<double, 12> P_pred_top_rows = extract_first_three_rows(P_pred);
    const std::array<double, 16> KPHt = mat4x3_mul_3x4(K, P_pred_top_rows);
    P_ = mat4_sub(P_pred, KPHt);

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
    out.pose.orientation = quaternion_from_yaw(state_[2]);
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
    marker.ns = "ekf_covariance";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.06;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.4;
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
    marker.ns = "ekf_pose_arrow";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose = pose.pose;
    marker.scale.x = 0.6;
    marker.scale.y = 0.12;
    marker.scale.z = 0.12;
    marker.color.r = 0.2;
    marker.color.g = 0.9;
    marker.color.b = 0.2;
    marker.color.a = 0.9;
    return marker;
  }

  static double normalize_angle(double angle)
  {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
  }

  static double yaw_from_quaternion(const geometry_msgs::msg::Quaternion &q)
  {
    const double siny = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny, cosy);
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

  static std::array<double, 9> extract_top_left_3x3(const std::array<double, 16> &m)
  {
    return { m[0], m[1], m[2], m[4], m[5], m[6], m[8], m[9], m[10] };
  }

  static std::array<double, 12> extract_first_three_columns(const std::array<double, 16> &m)
  {
    std::array<double, 12> out{};
    for (int row = 0; row < 4; ++row) {
      for (int col = 0; col < 3; ++col) {
        out[row * 3 + col] = m[row * 4 + col];
      }
    }
    return out;
  }

  static std::array<double, 12> extract_first_three_rows(const std::array<double, 16> &m)
  {
    std::array<double, 12> out{};
    for (int row = 0; row < 3; ++row) {
      for (int col = 0; col < 4; ++col) {
        out[row * 4 + col] = m[row * 4 + col];
      }
    }
    return out;
  }

  static std::array<double, 12> mat4x3_mul_3x3(const std::array<double, 12> &a, const std::array<double, 9> &b)
  {
    std::array<double, 12> out{};
    for (int row = 0; row < 4; ++row) {
      for (int col = 0; col < 3; ++col) {
        double sum = 0.0;
        for (int k = 0; k < 3; ++k) {
          sum += a[row * 3 + k] * b[k * 3 + col];
        }
        out[row * 3 + col] = sum;
      }
    }
    return out;
  }

  static std::array<double, 4> mat4x3_vec_mul(const std::array<double, 12> &m, const std::array<double, 3> &v)
  {
    std::array<double, 4> out{};
    for (int row = 0; row < 4; ++row) {
      out[row] = m[row * 3 + 0] * v[0] + m[row * 3 + 1] * v[1] + m[row * 3 + 2] * v[2];
    }
    return out;
  }

  static std::array<double, 16> mat4x3_mul_3x4(const std::array<double, 12> &a, const std::array<double, 12> &b)
  {
    std::array<double, 16> out{};
    for (int row = 0; row < 4; ++row) {
      for (int col = 0; col < 4; ++col) {
        double sum = 0.0;
        for (int k = 0; k < 3; ++k) {
          sum += a[row * 3 + k] * b[k * 4 + col];
        }
        out[row * 4 + col] = sum;
      }
    }
    return out;
  }

  static std::array<double, 9> mat3_add(const std::array<double, 9> &a, const std::array<double, 9> &b)
  {
    std::array<double, 9> out;
    for (int i = 0; i < 9; ++i) {
      out[i] = a[i] + b[i];
    }
    return out;
  }

  static std::array<double, 9> invert3(const std::array<double, 9> &m)
  {
    const double a = m[0]; const double b = m[1]; const double c = m[2];
    const double d = m[3]; const double e = m[4]; const double f = m[5];
    const double g = m[6]; const double h = m[7]; const double i = m[8];
    const double det = a*(e*i - f*h) - b*(d*i - f*g) + c*(d*h - e*g);
    const double inv_det = std::fabs(det) < 1e-12 ? 1e12 : 1.0 / det;
    return {
      (e * i - f * h) * inv_det,
      (c * h - b * i) * inv_det,
      (b * f - c * e) * inv_det,
      (f * g - d * i) * inv_det,
      (a * i - c * g) * inv_det,
      (c * d - a * f) * inv_det,
      (d * h - e * g) * inv_det,
      (b * g - a * h) * inv_det,
      (a * e - b * d) * inv_det
    };
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
  std::array<double, 9> R_;
  bool initialized_ = false;
  rclcpp::Time last_time_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EKFNode>());
  rclcpp::shutdown();
  return 0;
}
