/*
 * Copyright 2024 Manifold Tech.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <cmath>
#include <memory>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

/**
 * @brief Adapts Odin PointXYZRGB cloud to PointXYZI with optional min-range filter.
 *
 * Odin publishes /odin1/cloud_slam as PointXYZRGB (fields: x, y, z, rgb).
 * CMU planner modules (terrain_analysis, terrain_analysis_ext) expect
 * /registered_scan as PointXYZI (fields: x, y, z, intensity).
 *
 * This node:
 *   1. Subscribes to the input cloud and /state_estimation (vehicle pose).
 *   2. Filters points closer than scan_min_range from the vehicle.
 *   3. Converts PointXYZRGB → PointXYZI (drops rgb, sets intensity=0.0f).
 *   4. Publishes to /registered_scan, preserving the original header.
 *
 * The vehicle-relative distance filter is needed because cloud_slam points
 * are in the 'odin_odom' frame (global coordinates), not sensor-relative.
 */
class RegisteredScanAdapterNode : public rclcpp::Node
{
public:
  explicit RegisteredScanAdapterNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("registered_scan_adapter_node", options),
    odom_received_(false),
    vehicle_x_(0.0),
    vehicle_y_(0.0),
    vehicle_z_(0.0)
  {
    // Declare and read parameters
    scan_min_range_ =
      this->declare_parameter<double>("scan_min_range", 0.0);
    input_topic_ =
      this->declare_parameter<std::string>("input_topic", "/odin1/cloud_slam");
    output_topic_ =
      this->declare_parameter<std::string>("output_topic", "/registered_scan");
    state_topic_ =
      this->declare_parameter<std::string>("state_topic", "/state_estimation");

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

    // Subscriber for vehicle odometry (to compute vehicle-relative distance)
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      state_topic_,
      qos,
      std::bind(&RegisteredScanAdapterNode::odomCallback, this, std::placeholders::_1));

    // Subscriber for input point cloud
    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_,
      qos,
      std::bind(&RegisteredScanAdapterNode::cloudCallback, this, std::placeholders::_1));

    // Publisher for output point cloud
    cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, qos);

    RCLCPP_INFO(
      this->get_logger(),
      "registered_scan_adapter_node: %s → %s, state_topic=%s, scan_min_range=%.3f m",
      input_topic_.c_str(),
      output_topic_.c_str(),
      state_topic_.c_str(),
      scan_min_range_);
  }

private:
  /**
   * @brief Caches the latest vehicle pose from /state_estimation.
   */
  void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    vehicle_x_ = msg->pose.pose.position.x;
    vehicle_y_ = msg->pose.pose.position.y;
    vehicle_z_ = msg->pose.pose.position.z;
    odom_received_ = true;
  }

  /**
   * @brief Filters input cloud by vehicle-relative distance and converts XYZRGB → XYZI.
   */
  void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
  {
    // --- Step 1: Get latest vehicle position (thread-safe) ---
    double vx, vy, vz;
    bool have_odom;
    {
      std::lock_guard<std::mutex> lock(odom_mutex_);
      vx = vehicle_x_;
      vy = vehicle_y_;
      vz = vehicle_z_;
      have_odom = odom_received_;
    }

    // --- Step 2: Convert input PointCloud2 → PCL PointXYZRGB ---
    pcl::PointCloud<pcl::PointXYZRGB> cloud_in;
    pcl::fromROSMsg(*msg, cloud_in);

    if (cloud_in.empty()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "Received empty point cloud, skipping");
      return;
    }

    // --- Step 3: Filter by vehicle-relative distance ---
    // Compute squared min range once for efficiency
    const float min_range_sq =
      static_cast<float>(scan_min_range_ * scan_min_range_);
    // Only apply range filter if we have odometry AND min_range > 0
    const bool apply_range_filter = have_odom && (scan_min_range_ > 0.0);

    pcl::PointCloud<pcl::PointXYZRGB> cloud_filtered;
    cloud_filtered.header = cloud_in.header;
    cloud_filtered.is_dense = false;
    cloud_filtered.points.reserve(cloud_in.points.size());

    for (const auto & pt : cloud_in.points) {
      // Skip NaN/Inf points — they cause issues downstream
      if (!std::isfinite(pt.x) ||
          !std::isfinite(pt.y) ||
          !std::isfinite(pt.z)) {
        continue;
      }

      if (apply_range_filter) {
        // Horizontal distance from vehicle (matches terrain_analysis approach)
        const float dx = pt.x - static_cast<float>(vx);
        const float dy = pt.y - static_cast<float>(vy);
        const float dist_sq = dx * dx + dy * dy;

        if (dist_sq < min_range_sq) {
          continue;  // too close to vehicle, skip
        }
      }

      cloud_filtered.points.push_back(pt);
    }

    // --- Step 4: Convert PointXYZRGB → PointXYZI ---
    pcl::PointCloud<pcl::PointXYZI> cloud_out;
    cloud_out.header = cloud_filtered.header;
    cloud_out.is_dense = cloud_filtered.is_dense;
    cloud_out.points.reserve(cloud_filtered.points.size());

    for (const auto & pt : cloud_filtered.points) {
      pcl::PointXYZI pt_i;
      pt_i.x = pt.x;
      pt_i.y = pt.y;
      pt_i.z = pt.z;
      // terrain_analysis immediately overwrites intensity with
      // (laserCloudTime - systemInitTime), so the initial value is irrelevant.
      pt_i.intensity = 0.0f;
      cloud_out.points.push_back(pt_i);
    }

    cloud_out.width = static_cast<uint32_t>(cloud_out.points.size());
    cloud_out.height = 1;

    // --- Step 5: Publish with preserved header ---
    sensor_msgs::msg::PointCloud2 output_msg;
    pcl::toROSMsg(cloud_out, output_msg);

    // Preserve input timestamp and frame_id (both needed downstream)
    output_msg.header = msg->header;

    cloud_pub_->publish(output_msg);
  }

  // Parameters
  double scan_min_range_;
  std::string input_topic_;
  std::string output_topic_;
  std::string state_topic_;

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  // Publisher
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;

  // Latest vehicle position (odom frame), mutex-protected
  std::mutex odom_mutex_;
  bool odom_received_;
  double vehicle_x_;
  double vehicle_y_;
  double vehicle_z_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RegisteredScanAdapterNode>();
  RCLCPP_INFO(node->get_logger(), "registered_scan_adapter_node started");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
