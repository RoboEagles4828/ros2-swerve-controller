// Copyright 2021 ros2_control Development Team
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

#ifndef SWERVE_HARDWARE__ISAAC_DRIVE_HPP_
#define SWERVE_HARDWARE__ISAAC_DRIVE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <map>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "swerve_hardware/visibility_control.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "realtime_tools/realtime_box.h"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace swerve_hardware
{
class IsaacDriveHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(IsaacDriveHardware)

  SWERVE_HARDWARE_PUBLIC
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  SWERVE_HARDWARE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  SWERVE_HARDWARE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  SWERVE_HARDWARE_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  SWERVE_HARDWARE_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  SWERVE_HARDWARE_PUBLIC
  hardware_interface::return_type read() override;

  SWERVE_HARDWARE_PUBLIC
  hardware_interface::return_type write() override;

private:
  // Parameters for the DiffBot simulation
  double hw_start_sec_;
  double hw_stop_sec_;

  // Store the command for the simulated robot
  std::vector<double> hw_command_velocity_;
  // std::vector<double> hw_command_position_;
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> empty_;
  std::vector<std::string> joint_names_;
  std::vector<std::string> joint_names_velocity_;
  std::vector<std::string> joint_names_position_;

  std::map<std::string, uint> joint_names_map_;


  // Pub Sub to isaac
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::JointState>> isaac_publisher_ = nullptr;
  std::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>>
    realtime_isaac_publisher_ = nullptr;

  bool subscriber_is_active_ = false;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr isaac_subscriber_ = nullptr;
  realtime_tools::RealtimeBox<std::shared_ptr<sensor_msgs::msg::JointState>> received_joint_msg_ptr_{nullptr};
};

}  // namespace swerve_hardware

#endif  // SWERVE_HARDWARE__DIFFBOT_SYSTEM_HPP_