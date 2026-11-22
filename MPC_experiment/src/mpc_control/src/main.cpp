#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <fstream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

#include "mpc_controller.h"
#include "reference_line.h"

namespace {

class MpcControlNode final : public rclcpp::Node {
 public:
  MpcControlNode()
      : Node("mpc_control"), controller_(std::make_unique<shenlan::control::MPCController>()) {
    const auto package_share = ament_index_cpp::get_package_share_directory("mpc_control");
    const auto default_reference_line = package_share + "/data/reference_line.txt";

    reference_line_path_ = declare_parameter<std::string>("reference_line_path", default_reference_line);
    simulate_ = declare_parameter<bool>("simulate", true);
    odometry_topic_ = declare_parameter<std::string>("odometry_topic", "/odom");
    imu_topic_ = declare_parameter<std::string>("imu_topic", "/imu");
    control_topic_ = declare_parameter<std::string>("control_topic", "/mpc/control_cmd");
    control_frequency_hz_ = declare_parameter<double>("control_frequency_hz", 100.0);
    max_steering_angle_rad_ = declare_parameter<double>("max_steering_angle_rad", M_PI / 6.0);
    max_acceleration_ = declare_parameter<double>("max_acceleration", 0.8);
    simulation_wheelbase_ = declare_parameter<double>("simulation_wheelbase", 1.0);
    simulation_initial_speed_ = declare_parameter<double>("simulation_initial_speed", 0.0);
    visualization_frame_ = declare_parameter<std::string>("visualization_frame", "map");
    reference_line_topic_ = declare_parameter<std::string>("reference_line_topic", "mpc/reference_line");
    output_trajectory_topic_ = declare_parameter<std::string>("output_trajectory_topic", "mpc/output_trajectory");
    output_trajectory_size_ = declare_parameter<int>("output_trajectory_size", 2000);

    if (control_frequency_hz_ <= 0.0 || max_steering_angle_rad_ <= 0.0 || max_acceleration_ <= 0.0 ||
        simulation_wheelbase_ <= 0.0 || output_trajectory_size_ <= 0) {
      throw std::invalid_argument("control frequency, vehicle parameters, and trajectory size must be positive");
    }

    LoadReferenceLine();
    controller_->Init();

    if (!simulate_) {
      odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
        odometry_topic_, rclcpp::SensorDataQoS(),
        std::bind(&MpcControlNode::OnOdometry, this, std::placeholders::_1));
      imu_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_, rclcpp::SensorDataQoS(),
        std::bind(&MpcControlNode::OnImu, this, std::placeholders::_1));
    }
    control_publisher_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(control_topic_, 10);
    reference_line_publisher_ = create_publisher<nav_msgs::msg::Path>(
      reference_line_topic_, rclcpp::QoS(1).reliable().transient_local());
    output_trajectory_publisher_ = create_publisher<nav_msgs::msg::Path>(output_trajectory_topic_, 10);
    PublishReferenceLine();
    if (simulate_) {
      InitializeSimulation();
    }

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / control_frequency_hz_));
    control_timer_ = create_wall_timer(period, std::bind(&MpcControlNode::OnControlTimer, this));

    RCLCPP_INFO(get_logger(), "MPC controller initialized in %s mode with reference line: %s",
          simulate_ ? "simulation" : "external-input", reference_line_path_.c_str());
  }

 private:
  void LoadReferenceLine() {
    std::ifstream input(reference_line_path_);
    if (!input.is_open()) {
      throw std::runtime_error("Unable to open reference line: " + reference_line_path_);
    }

    std::vector<std::pair<double, double>> points;
    std::string line;
    while (std::getline(input, line)) {
      std::istringstream values(line);
      double x = 0.0;
      double y = 0.0;
      if (values >> x >> y) {
        const bool is_duplicate = !points.empty() &&
            std::hypot(x - points.back().first, y - points.back().second) < 1e-6;
        if (!is_duplicate) {
          points.emplace_back(x, y);
        }
      }
    }

    if (points.size() < 2) {
      throw std::runtime_error("Reference line must contain at least two valid points");
    }

    shenlan::control::ReferenceLine reference_line(points);
    std::vector<double> headings;
    std::vector<double> accumulated_s;
    std::vector<double> kappas;
    std::vector<double> dkappas;
    reference_line.ComputePathProfile(&headings, &accumulated_s, &kappas, &dkappas);

    trajectory_.trajectory_points.reserve(points.size());
    for (std::size_t index = 0; index < points.size(); ++index) {
      TrajectoryPoint point;
      point.x = points[index].first;
      point.y = points[index].second;
      point.v = 5.0;
      point.a = 0.0;
      point.heading = headings[index];
      point.kappa = kappas[index];
      trajectory_.trajectory_points.push_back(point);
    }
  }

  void OnOdometry(const nav_msgs::msg::Odometry::SharedPtr message) {
    tf2::Quaternion orientation(
        message->pose.pose.orientation.x,
        message->pose.pose.orientation.y,
        message->pose.pose.orientation.z,
        message->pose.pose.orientation.w);
    tf2::Matrix3x3(orientation).getRPY(vehicle_state_.roll, vehicle_state_.pitch, vehicle_state_.yaw);

    vehicle_state_.x = message->pose.pose.position.x;
    vehicle_state_.y = message->pose.pose.position.y;
    vehicle_state_.heading = vehicle_state_.yaw;
    vehicle_state_.vx = message->twist.twist.linear.x;
    vehicle_state_.vy = message->twist.twist.linear.y;
    vehicle_state_.velocity = std::hypot(vehicle_state_.vx, vehicle_state_.vy);
    has_odometry_ = true;
    AppendOutputTrajectory(rclcpp::Time(message->header.stamp));
  }

  void OnImu(const sensor_msgs::msg::Imu::SharedPtr message) {
    vehicle_state_.angular_velocity = message->angular_velocity.z;
    vehicle_state_.acceleration = std::hypot(message->linear_acceleration.x, message->linear_acceleration.y);
  }

  void PublishReferenceLine() {
    nav_msgs::msg::Path reference_line;
    reference_line.header.stamp = now();
    reference_line.header.frame_id = visualization_frame_;
    reference_line.poses.reserve(trajectory_.trajectory_points.size());

    for (const auto &point : trajectory_.trajectory_points) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = reference_line.header;
      pose.pose.position.x = point.x;
      pose.pose.position.y = point.y;
      tf2::Quaternion orientation;
      orientation.setRPY(0.0, 0.0, point.heading);
      pose.pose.orientation.x = orientation.x();
      pose.pose.orientation.y = orientation.y();
      pose.pose.orientation.z = orientation.z();
      pose.pose.orientation.w = orientation.w();
      reference_line.poses.push_back(pose);
    }

    reference_line_publisher_->publish(reference_line);
  }

  void InitializeSimulation() {
    const auto &start = trajectory_.trajectory_points.front();
    vehicle_state_.x = start.x;
    vehicle_state_.y = start.y;
    vehicle_state_.heading = start.heading;
    vehicle_state_.yaw = start.heading;
    vehicle_state_.velocity = simulation_initial_speed_;
    vehicle_state_.vx = simulation_initial_speed_ * std::cos(start.heading);
    vehicle_state_.vy = simulation_initial_speed_ * std::sin(start.heading);
    has_odometry_ = true;
    AppendOutputTrajectory(now());
  }

  void SimulateVehicle(const ControlCmd &command) {
    const double period = 1.0 / control_frequency_hz_;
    const double steering = std::clamp(command.steer_target, -max_steering_angle_rad_, max_steering_angle_rad_);
    const double acceleration = std::clamp(command.acc, -max_acceleration_, max_acceleration_);
    const double next_speed = std::max(0.0, vehicle_state_.velocity + acceleration * period);
    const double yaw_rate = next_speed * std::tan(steering) / simulation_wheelbase_;

    vehicle_state_.heading += yaw_rate * period;
    vehicle_state_.yaw = vehicle_state_.heading;
    vehicle_state_.velocity = next_speed;
    vehicle_state_.angular_velocity = yaw_rate;
    vehicle_state_.acceleration = acceleration;
    vehicle_state_.vx = next_speed * std::cos(vehicle_state_.heading);
    vehicle_state_.vy = next_speed * std::sin(vehicle_state_.heading);
    vehicle_state_.x += vehicle_state_.vx * period;
    vehicle_state_.y += vehicle_state_.vy * period;
    AppendOutputTrajectory(now());
  }

  void AppendOutputTrajectory(const rclcpp::Time &stamp) {
    geometry_msgs::msg::PoseStamped output_pose;
    output_pose.header.stamp = stamp;
    output_pose.header.frame_id = visualization_frame_;
    output_pose.pose.position.x = vehicle_state_.x;
    output_pose.pose.position.y = vehicle_state_.y;
    tf2::Quaternion orientation;
    orientation.setRPY(0.0, 0.0, vehicle_state_.heading);
    output_pose.pose.orientation.x = orientation.x();
    output_pose.pose.orientation.y = orientation.y();
    output_pose.pose.orientation.z = orientation.z();
    output_pose.pose.orientation.w = orientation.w();

    output_trajectory_.header.stamp = stamp;
    output_trajectory_.header.frame_id = visualization_frame_;
    output_trajectory_.poses.push_back(output_pose);
    if (output_trajectory_.poses.size() > static_cast<std::size_t>(output_trajectory_size_)) {
      output_trajectory_.poses.erase(output_trajectory_.poses.begin());
    }
    output_trajectory_publisher_->publish(output_trajectory_);
  }

  void OnControlTimer() {
    if (!has_odometry_) {
      return;
    }

    ControlCmd command{};
    if (!controller_->ComputeControlCommand(vehicle_state_, trajectory_, command) ||
        !std::isfinite(command.steer_target) || !std::isfinite(command.acc)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "MPC solver did not produce a valid command");
      return;
    }

    geometry_msgs::msg::Vector3Stamped control;
    control.header.stamp = now();
    control.header.frame_id = visualization_frame_;
    control.vector.x = command.acc;
    control.vector.z = command.steer_target;
    control_publisher_->publish(control);

    if (simulate_) {
      SimulateVehicle(command);
    }
  }

  std::unique_ptr<shenlan::control::MPCController> controller_;
  VehicleState vehicle_state_;
  TrajectoryData trajectory_;
  bool has_odometry_ = false;
  bool simulate_ = true;
  std::string reference_line_path_;
  std::string odometry_topic_;
  std::string imu_topic_;
  std::string control_topic_;
  std::string visualization_frame_;
  std::string reference_line_topic_;
  std::string output_trajectory_topic_;
  double control_frequency_hz_ = 100.0;
  double max_steering_angle_rad_ = M_PI / 6.0;
  double max_acceleration_ = 0.8;
  double simulation_wheelbase_ = 1.0;
  double simulation_initial_speed_ = 0.0;
  int output_trajectory_size_ = 2000;
  nav_msgs::msg::Path output_trajectory_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr control_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr reference_line_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr output_trajectory_publisher_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

}  // namespace

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MpcControlNode>());
  rclcpp::shutdown();
  return 0;
}