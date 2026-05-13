// Copyright (c) 2026 Masazumi Imai
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace mlivr_sim
{
class EnvVisualizer : public rclcpp::Node
{
public:
  EnvVisualizer(const rclcpp::NodeOptions & options) : Node("env_visualizer", options)
  {
    // ROS 2 parameters
    std::string mesh_resource = this->declare_parameter<std::string>(
      "mesh_resource", "package://mlivr_description/meshes/iss_jem/iss_jem.stl");
    double scale = this->declare_parameter<double>("scale", 0.1);

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/iss_environment", qos);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1000),
      [this, mesh_resource, scale]() { this->publish_environment(mesh_resource, scale); });

    RCLCPP_INFO(
      this->get_logger(), "Environment Visualizer started. Mesh: %s", mesh_resource.c_str());
  }

private:
  void publish_environment(const std::string & resource, double scale)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = this->now();
    marker.ns = "iss_jem";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.mesh_resource = resource;
    marker.mesh_use_embedded_materials = true;  // Use textures in DAE

    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = scale;
    marker.scale.y = scale;
    marker.scale.z = scale;

    // Backup color in case of no color in DAE
    marker.color.a = 1.0;
    marker.color.r = 0.5;
    marker.color.g = 0.5;
    marker.color.b = 0.5;

    marker_pub_->publish(marker);
  }

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace mlivr_sim

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<mlivr_sim::EnvVisualizer>(rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
