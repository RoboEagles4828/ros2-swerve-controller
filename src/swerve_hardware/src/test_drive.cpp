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

#include "swerve_hardware/test_drive.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
using std::placeholders::_1;

namespace swerve_hardware
{
CallbackReturn TestDriveHardware::on_init(const hardware_interface::HardwareInfo & info)
{

  // INTERFACE SETUP
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  // 8 positions states, 4 axle positions 4 wheel positions
  hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  // 8 velocity states, 4 axle velocity 4 wheel velocity
  hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  
  // 4 wheel velocity commands 
  hw_command_velocity_.resize(info_.joints.size()/2, std::numeric_limits<double>::quiet_NaN());
  // 4 axle position commands
  hw_command_position_.resize(info_.joints.size()/2, std::numeric_limits<double>::quiet_NaN());


  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    joint_names_.push_back(joint.name);
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("TestDriveHardware"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY && joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("TestDriveHardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("TestDriveHardware"),
        "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("TestDriveHardware"),
        "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("TestDriveHardware"),
        "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return CallbackReturn::ERROR;
    }
  }

  return CallbackReturn::SUCCESS;
}



std::vector<hardware_interface::StateInterface> TestDriveHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
  }

  return state_interfaces;
}



std::vector<hardware_interface::CommandInterface> TestDriveHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  uint counter_position =0;
  uint counter_velocity =0;

  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    auto joint = info_.joints[i];
    RCLCPP_INFO(rclcpp::get_logger("TestDriveHardware"), "Joint Name %s", joint.name.c_str());

    if (joint.command_interfaces[0].name == hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_INFO(rclcpp::get_logger("TestDriveHardware"), "Added Velocity Joint: %s", joint.name.c_str() );
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint.name, hardware_interface::HW_IF_VELOCITY, &hw_command_velocity_[counter_velocity]));

      names_to_vel_cmd_map_[joint.name] = counter_velocity + 1; // ADD 1 to differentiate from a key that is not found
      counter_velocity++;
    } else {
      RCLCPP_INFO(rclcpp::get_logger("TestDriveHardware"), "Added Position Joint: %s", joint.name.c_str() );
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint.name, hardware_interface::HW_IF_POSITION, &hw_command_position_[counter_position]));
      
      names_to_pos_cmd_map_[joint.name] = counter_position + 1; // ADD 1 to differentiate from a key that is not found
      counter_position++;
    }
  }

  return command_interfaces;
}



CallbackReturn TestDriveHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("TestDriveHardware"), "Activating ...please wait...");

  // Set Default Values for State Interface Arrays
  for (auto i = 0u; i < hw_positions_.size(); i++)
  {
    hw_positions_[i] = 0.0;
    hw_velocities_[i] = 0.0;
  }

  // Set Default Values for Command Interface Arrays
  for (auto i = 0u; i < hw_command_velocity_.size(); i++)
  {
    hw_command_velocity_[i] = 0.0;
    hw_command_position_[i] = 0.0;
  }

  RCLCPP_INFO(rclcpp::get_logger("TestDriveHardware"), "Successfully activated!");

  return CallbackReturn::SUCCESS;
}



CallbackReturn TestDriveHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("TestDriveHardware"), "Deactivating ...please wait...");
  RCLCPP_INFO(rclcpp::get_logger("TestDriveHardware"), "Successfully deactivated!");

  return CallbackReturn::SUCCESS;
}


// ||                        ||
// \/ THE STUFF THAT MATTERS \/

hardware_interface::return_type TestDriveHardware::read()
{

  // Dumb Pass Through
  // If you give the command for x velocity then the state is x velocity

  // Loop through each joint name
  // Check if joint name is in velocity command map
  // If it is, use the index from the map to get the value in the velocity array
  // If velocity not in map, set velocity value to 0
  // Perform the same for position

  double dt = 0.01;
  for (auto i = 0u; i < joint_names_.size(); i++) 
  {
    auto vel_i = names_to_vel_cmd_map_[joint_names_[i]];
    auto pos_i = names_to_pos_cmd_map_[joint_names_[i]];
    
    if (vel_i > 0) {
      // RCLCPP_INFO(rclcpp::get_logger("TestDriveHardware"), "J %f", hw_command_velocity_[vel_i - 1]);
      auto vel = hw_command_velocity_[vel_i - 1];
      hw_velocities_[i] = vel;
      hw_positions_[i] = hw_positions_[i] + dt * vel;
    } else if (pos_i > 0) {
      auto pos = hw_command_position_[pos_i - 1];
      hw_velocities_[i] = 0.0;
      hw_positions_[i] = pos;
    }
  }
  
  return hardware_interface::return_type::OK;
}



hardware_interface::return_type swerve_hardware::TestDriveHardware::write()
{
  // Do Nothing
  // Uncomment below if you want verbose messages for debugging.
  for (auto i = 0u; i < hw_command_velocity_.size(); i++)
  {
    RCLCPP_INFO(rclcpp::get_logger("TestDriveHardware"), "Wheel %u Velocity: %f", i, hw_command_velocity_[i]);
  }

  return hardware_interface::return_type::OK;
}

}  // namespace swerve_hardware



#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  swerve_hardware::TestDriveHardware, hardware_interface::SystemInterface)