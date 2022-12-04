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

#ifndef SWERVE_HARDWARE__TEST_DRIVE_HPP_
#define SWERVE_HARDWARE__TEST_DRIVE_HPP_

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

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace swerve_hardware
{
class TestDriveHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(TestDriveHardware)

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
  // Store the command for the simulated robot
  std::vector<double> hw_command_velocity_;
  std::vector<double> hw_command_position_;

  // Map for easy lookup name -> joint command value
  std::map<std::string, uint> names_to_vel_cmd_map_;
  std::map<std::string, uint> names_to_pos_cmd_map_;

  // The state vectors
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;

  // Joint name array will align with state and command interface array
  // The command at index 3 of hw_command_ will be the joint name at index 3 of joint_names
  std::vector<std::string> joint_names_;
};

}  // namespace swerve_hardware

#endif  // SWERVE_HARDWARE__TEST_DRIVE_HPP_