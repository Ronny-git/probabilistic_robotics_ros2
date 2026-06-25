#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <cmath>
#include <memory>
#include <random>
#include <vector>

struct Particle
{
  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;
  double weight = 1.0;
};

class PFNode : public rclcpp::Node
{
public:
  PFNode() : Node("pf_node"), generator_(std::random_device{}())
  {
    sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&PFNode::odom_callback, this, std::placeholders::_1));

    pub_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/pf/pose", 10);
    pub_path_ = this->create_publisher<nav_msgs::msg::Path>("/pf/path", 10);
    pub_cov_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "/pf/covariance", 10);
    pub_arrow_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "/pf/pose_arrow", 10);

    path_.header.frame_id = "map";
    particles_.resize(300);
    noise_x_ = std::normal_distribution<double>(0.0, 0.02);
    noise_y_ = std::normal_distribution<double>(0.0, 0.02);
    noise_theta_ = std::normal_distribution<double>(0.0, 0.01);
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
      initialize_particles(meas_x, meas_y, meas_theta);
      last_time_ = stamp;
      initialized_ = true;
      publish_mean(stamp);
      return;
    }

    const double dt = std::max(0.001, (stamp - last_time_).seconds());
    last_time_ = stamp;

    for (auto &particle : particles_) {
      const double noisy_v = lin_v + noise_x_(generator_);
      const double noisy_omega = ang_v + noise_theta_(generator_);
      particle.x += noisy_v * std::cos(particle.theta) * dt + noise_x_(generator_);
      particle.y += noisy_v * std::sin(particle.theta) * dt + noise_y_(generator_);
      particle.theta = normalize_angle(particle.theta + noisy_omega * dt + noise_theta_(generator_));
    }

    double total_weight = 0.0;
    for (auto &particle : particles_) {
      const double dx = meas_x - particle.x;
      const double dy = meas_y - particle.y;
      double dtheta = normalize_angle(meas_theta - particle.theta);
      const double pos_err = dx * dx + dy * dy;
      const double ang_err = dtheta * dtheta;
      const double measure_sigma = 0.15;
      const double weight = std::exp(-0.5 * ((pos_err / (measure_sigma * measure_sigma)) + (ang_err / (0.3 * 0.3))));
      particle.weight = std::max(1e-6, weight);
      total_weight += particle.weight;
    }

    for (auto &particle : particles_) {
      particle.weight /= total_weight;
    }

    if (effective_sample_size() < static_cast<double>(particles_.size()) * 0.5) {
      resample_particles();
    }

    publish_mean(stamp);
  }

  void initialize_particles(double x, double y, double theta)
  {
    std::normal_distribution<double> init_x(0.0, 0.1);
    std::normal_distribution<double> init_y(0.0, 0.1);
    std::normal_distribution<double> init_theta(0.0, 0.05);
    for (auto &particle : particles_) {
      particle.x = x + init_x(generator_);
      particle.y = y + init_y(generator_);
      particle.theta = normalize_angle(theta + init_theta(generator_));
      particle.weight = 1.0 / particles_.size();
    }
  }

  double effective_sample_size() const
  {
    double sum_sq = 0.0;
    for (const auto &particle : particles_) {
      sum_sq += particle.weight * particle.weight;
    }
    return 1.0 / std::max(sum_sq, 1e-12);
  }

  void resample_particles()
  {
    std::vector<Particle> new_particles;
    new_particles.reserve(particles_.size());

    std::uniform_real_distribution<double> dist(0.0, 1.0 / particles_.size());
    double r = dist(generator_);
    double c = particles_[0].weight;
    std::size_t i = 0;
    for (std::size_t m = 0; m < particles_.size(); ++m) {
      const double u = r + m * (1.0 / particles_.size());
      while (u > c && i + 1 < particles_.size()) {
        ++i;
        c += particles_[i].weight;
      }
      new_particles.push_back(particles_[i]);
      new_particles.back().weight = 1.0 / particles_.size();
    }
    particles_.swap(new_particles);
  }

  void publish_mean(const rclcpp::Time &stamp)
  {
    double mean_x = 0.0;
    double mean_y = 0.0;
    double mean_sin = 0.0;
    double mean_cos = 0.0;

    for (const auto &particle : particles_) {
      mean_x += particle.x * particle.weight;
      mean_y += particle.y * particle.weight;
      mean_sin += std::sin(particle.theta) * particle.weight;
      mean_cos += std::cos(particle.theta) * particle.weight;
    }
    const double mean_theta = std::atan2(mean_sin, mean_cos);

    geometry_msgs::msg::PoseStamped out;
    out.header.stamp = stamp;
    out.header.frame_id = "map";
    out.pose.position.x = mean_x;
    out.pose.position.y = mean_y;
    out.pose.position.z = 0.0;
    out.pose.orientation = quaternion_from_yaw(mean_theta);
    pub_pose_->publish(out);

    path_.header.stamp = stamp;
    path_.poses.push_back(out);
    pub_path_->publish(path_);

    const auto cov = compute_covariance(mean_x, mean_y);
    pub_cov_->publish(make_covariance_marker(out, cov));
    pub_arrow_->publish(make_arrow_marker(out));
  }

  std::array<double, 3> compute_covariance(double mean_x, double mean_y) const
  {
    double cov_xx = 0.0;
    double cov_xy = 0.0;
    double cov_yy = 0.0;
    for (const auto &particle : particles_) {
      const double dx = particle.x - mean_x;
      const double dy = particle.y - mean_y;
      cov_xx += particle.weight * dx * dx;
      cov_xy += particle.weight * dx * dy;
      cov_yy += particle.weight * dy * dy;
    }
    return {cov_xx, cov_xy, cov_yy};
  }

  visualization_msgs::msg::Marker make_covariance_marker(
      const geometry_msgs::msg::PoseStamped &pose,
      const std::array<double, 3> &cov) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header = pose.header;
    marker.ns = "pf_covariance";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.06;
    marker.color.r = 0.9;
    marker.color.g = 0.4;
    marker.color.b = 0.1;
    marker.color.a = 0.9;

    const double a = cov[0];
    const double b = cov[1];
    const double c = cov[2];
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
    marker.ns = "pf_pose_arrow";
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

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_cov_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_arrow_;
  nav_msgs::msg::Path path_;
  std::vector<Particle> particles_;
  std::default_random_engine generator_;
  std::normal_distribution<double> noise_x_;
  std::normal_distribution<double> noise_y_;
  std::normal_distribution<double> noise_theta_;
  bool initialized_ = false;
  rclcpp::Time last_time_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PFNode>());
  rclcpp::shutdown();
  return 0;
}
