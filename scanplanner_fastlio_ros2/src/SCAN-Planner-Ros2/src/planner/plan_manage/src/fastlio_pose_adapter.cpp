#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Eigen>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

namespace scan_planner
{
class FastlioPoseAdapter : public rclcpp::Node
{
public:
  FastlioPoseAdapter() : Node("fastlio_pose_adapter")
  {
    body_translation_ = vectorParameter("body_translation_in_imu", {0.0, 0.0, 0.0});
    body_rotation_ = rpyToQuaternion(vectorParameter("body_rpy_in_imu", {0.0, 0.0, 0.0}));
    sensor_translation_ = vectorParameter("sensor_translation_in_imu", {0.0, 0.0, 0.0});
    sensor_rotation_ = rpyToQuaternion(vectorParameter("sensor_rpy_in_imu", {0.0, 0.0, 0.0}));
    body_frame_ = declare_parameter<std::string>("body_frame", "base");
    sensor_frame_ = declare_parameter<std::string>("sensor_frame", "livox_frame");
    publish_tf_ = declare_parameter<bool>("publish_tf", true);
    publish_sensor_tf_ = declare_parameter<bool>("publish_sensor_tf", true);
    estimate_velocity_ = declare_parameter<bool>("estimate_velocity", true);
    velocity_filter_alpha_ = declare_parameter<double>("velocity_filter_alpha", 0.35);
    if (velocity_filter_alpha_ <= 0.0 || velocity_filter_alpha_ > 1.0)
      throw std::invalid_argument("velocity_filter_alpha must be in (0, 1]");

    const auto qos = rclcpp::SensorDataQoS();
    body_pub_ = create_publisher<nav_msgs::msg::Odometry>("body_pose", qos);
    sensor_pub_ = create_publisher<nav_msgs::msg::Odometry>("sensor_pose", qos);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "fastlio_odom", qos,
        std::bind(&FastlioPoseAdapter::odomCallback, this, std::placeholders::_1));
    if (publish_tf_)
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    if (publish_sensor_tf_)
    {
      if (body_frame_.empty() || sensor_frame_.empty())
        throw std::invalid_argument(
            "body_frame and sensor_frame must not be empty when publish_sensor_tf is true");
      if (body_frame_ == sensor_frame_)
        throw std::invalid_argument(
            "body_frame and sensor_frame must differ when publish_sensor_tf is true");
      static_tf_broadcaster_ =
          std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
      publishSensorTransform();
    }

    RCLCPP_INFO(
        get_logger(),
        "FAST-LIO pose adapter ready: IMU->%s [%.3f %.3f %.3f], IMU->%s [%.3f %.3f %.3f]",
        body_frame_.c_str(),
        body_translation_.x(), body_translation_.y(), body_translation_.z(),
        sensor_frame_.c_str(),
        sensor_translation_.x(), sensor_translation_.y(), sensor_translation_.z());
  }

private:
  Eigen::Vector3d vectorParameter(const std::string &name, const std::vector<double> &default_value)
  {
    const auto values = declare_parameter<std::vector<double>>(name, default_value);
    if (values.size() != 3)
      throw std::invalid_argument(name + " must contain exactly three values");
    return Eigen::Vector3d(values[0], values[1], values[2]);
  }

  static Eigen::Quaterniond rpyToQuaternion(const Eigen::Vector3d &rpy)
  {
    Eigen::Quaterniond quaternion =
        Eigen::AngleAxisd(rpy.z(), Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(rpy.y(), Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(rpy.x(), Eigen::Vector3d::UnitX());
    return quaternion.normalized();
  }

  static bool finitePose(const nav_msgs::msg::Odometry &message)
  {
    const auto &position = message.pose.pose.position;
    const auto &orientation = message.pose.pose.orientation;
    return std::isfinite(position.x) && std::isfinite(position.y) && std::isfinite(position.z) &&
           std::isfinite(orientation.x) && std::isfinite(orientation.y) &&
           std::isfinite(orientation.z) && std::isfinite(orientation.w);
  }

  nav_msgs::msg::Odometry transformPose(
      const nav_msgs::msg::Odometry &input, const Eigen::Vector3d &translation,
      const Eigen::Quaterniond &rotation, const std::string &child_frame) const
  {
    const auto &source_pose = input.pose.pose;
    Eigen::Quaterniond world_from_imu(
        source_pose.orientation.w, source_pose.orientation.x,
        source_pose.orientation.y, source_pose.orientation.z);
    world_from_imu.normalize();
    const Eigen::Vector3d imu_position(
        source_pose.position.x, source_pose.position.y, source_pose.position.z);
    const Eigen::Vector3d target_position = imu_position + world_from_imu * translation;
    const Eigen::Quaterniond world_from_target = (world_from_imu * rotation).normalized();

    nav_msgs::msg::Odometry output = input;
    output.child_frame_id = child_frame;
    output.pose.pose.position.x = target_position.x();
    output.pose.pose.position.y = target_position.y();
    output.pose.pose.position.z = target_position.z();
    output.pose.pose.orientation.x = world_from_target.x();
    output.pose.pose.orientation.y = world_from_target.y();
    output.pose.pose.orientation.z = world_from_target.z();
    output.pose.pose.orientation.w = world_from_target.w();
    return output;
  }

  void updateBodyVelocity(nav_msgs::msg::Odometry &body)
  {
    if (!estimate_velocity_)
      return;

    const rclcpp::Time stamp(body.header.stamp);
    const Eigen::Vector3d position(
        body.pose.pose.position.x, body.pose.pose.position.y, body.pose.pose.position.z);
    if (have_previous_pose_)
    {
      const double dt = (stamp - previous_stamp_).seconds();
      if (dt > 1e-3 && dt < 0.5)
      {
        const Eigen::Vector3d measured = (position - previous_body_position_) / dt;
        filtered_velocity_ = velocity_filter_alpha_ * measured +
                             (1.0 - velocity_filter_alpha_) * filtered_velocity_;
      }
    }
    previous_stamp_ = stamp;
    previous_body_position_ = position;
    have_previous_pose_ = true;
    // SCAN-Planner consumes odometry linear velocity in the world frame.
    body.twist.twist.linear.x = filtered_velocity_.x();
    body.twist.twist.linear.y = filtered_velocity_.y();
    body.twist.twist.linear.z = filtered_velocity_.z();
  }

  void publishBodyTransform(const nav_msgs::msg::Odometry &body)
  {
    if (!tf_broadcaster_ || body.header.frame_id.empty() || body.child_frame_id.empty())
      return;
    geometry_msgs::msg::TransformStamped transform;
    transform.header = body.header;
    transform.child_frame_id = body.child_frame_id;
    transform.transform.translation.x = body.pose.pose.position.x;
    transform.transform.translation.y = body.pose.pose.position.y;
    transform.transform.translation.z = body.pose.pose.position.z;
    transform.transform.rotation = body.pose.pose.orientation;
    tf_broadcaster_->sendTransform(transform);
  }

  void publishSensorTransform()
  {
    // Configured poses are T_imu_body and T_imu_sensor.  The robot TF tree
    // needs T_body_sensor = inverse(T_imu_body) * T_imu_sensor.
    const Eigen::Quaterniond body_from_imu = body_rotation_.conjugate();
    const Eigen::Vector3d sensor_translation_in_body =
        body_from_imu * (sensor_translation_ - body_translation_);
    const Eigen::Quaterniond sensor_rotation_in_body =
        (body_from_imu * sensor_rotation_).normalized();

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = now();
    transform.header.frame_id = body_frame_;
    transform.child_frame_id = sensor_frame_;
    transform.transform.translation.x = sensor_translation_in_body.x();
    transform.transform.translation.y = sensor_translation_in_body.y();
    transform.transform.translation.z = sensor_translation_in_body.z();
    transform.transform.rotation.x = sensor_rotation_in_body.x();
    transform.transform.rotation.y = sensor_rotation_in_body.y();
    transform.transform.rotation.z = sensor_rotation_in_body.z();
    transform.transform.rotation.w = sensor_rotation_in_body.w();
    static_tf_broadcaster_->sendTransform(transform);

    RCLCPP_INFO(
        get_logger(), "Publishing fixed TF %s -> %s [%.3f %.3f %.3f]",
        body_frame_.c_str(), sensor_frame_.c_str(), sensor_translation_in_body.x(),
        sensor_translation_in_body.y(), sensor_translation_in_body.z());
  }

  void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr input)
  {
    if (!finitePose(*input))
    {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000, "Ignoring non-finite FAST-LIO odometry");
      return;
    }
    const auto &orientation = input->pose.pose.orientation;
    const double quaternion_norm = std::sqrt(
        orientation.x * orientation.x + orientation.y * orientation.y +
        orientation.z * orientation.z + orientation.w * orientation.w);
    if (quaternion_norm < 1e-6)
    {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000, "Ignoring FAST-LIO odometry with invalid orientation");
      return;
    }

    auto body = transformPose(*input, body_translation_, body_rotation_, body_frame_);
    auto sensor = transformPose(*input, sensor_translation_, sensor_rotation_, sensor_frame_);
    updateBodyVelocity(body);
    body_pub_->publish(body);
    sensor_pub_->publish(sensor);
    publishBodyTransform(body);
  }

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr body_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr sensor_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
  Eigen::Vector3d body_translation_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d sensor_translation_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond body_rotation_{Eigen::Quaterniond::Identity()};
  Eigen::Quaterniond sensor_rotation_{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d previous_body_position_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d filtered_velocity_{Eigen::Vector3d::Zero()};
  rclcpp::Time previous_stamp_{0, 0, RCL_ROS_TIME};
  std::string body_frame_;
  std::string sensor_frame_;
  bool publish_tf_{true};
  bool publish_sensor_tf_{true};
  bool estimate_velocity_{true};
  bool have_previous_pose_{false};
  double velocity_filter_alpha_{0.35};
};
}  // namespace scan_planner

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  try
  {
    rclcpp::spin(std::make_shared<scan_planner::FastlioPoseAdapter>());
  }
  catch (const std::exception &error)
  {
    RCLCPP_FATAL(rclcpp::get_logger("fastlio_pose_adapter"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
