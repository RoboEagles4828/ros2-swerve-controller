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

#include "swerve_hardware/isaac_drive.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
using std::placeholders::_1;

namespace swerve_hardware
{
CallbackReturn IsaacDriveHardware::on_init(const hardware_interface::HardwareInfo & info)
{

  // rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;
  // custom_qos_profile.depth = 7;

  node_ = rclcpp::Node::make_shared("isaac_hardware_interface");

  // PUBLISHER SETUP
  isaac_publisher_ = node_->create_publisher<sensor_msgs::msg::JointState>("isaac_joint_commands", rclcpp::SystemDefaultsQoS());
  realtime_isaac_publisher_ = std::make_shared<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>>(
      isaac_publisher_);


  // SUBSCRIBER SETUP
  const sensor_msgs::msg::JointState empty_joint_state;
  received_joint_msg_ptr_.set(std::make_shared<sensor_msgs::msg::JointState>(empty_joint_state));
  isaac_subscriber_ = node_->create_subscription<sensor_msgs::msg::JointState>("isaac_joint_states", rclcpp::SystemDefaultsQoS(),
    [this](const std::shared_ptr<sensor_msgs::msg::JointState> msg) -> void
    {
      if (!subscriber_is_active_) {
        RCLCPP_WARN( rclcpp::get_logger("isaac_hardware_interface"), "Can't accept new commands. subscriber is inactive");
        return;
      }
      received_joint_msg_ptr_.set(std::move(msg));
    });


  // INTERFACE SETUP
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_command_velocity_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  // hw_command_position_.resize(info_.joints.size()/2, std::numeric_limits<double>::quiet_NaN());


  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    joint_names_.push_back(joint.name);
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("IsaacDriveHardware"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY && joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("IsaacDriveHardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("IsaacDriveHardware"),
        "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("IsaacDriveHardware"),
        "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("IsaacDriveHardware"),
        "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return CallbackReturn::ERROR;
    }
  }

  return CallbackReturn::SUCCESS;
}



std::vector<hardware_interface::StateInterface> IsaacDriveHardware::export_state_interfaces()
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



std::vector<hardware_interface::CommandInterface> IsaacDriveHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  // uint counter_position =0;
  // uint counter_velocity =0;

  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    RCLCPP_INFO(rclcpp::get_logger("IsaacDriveHardware"), "Joint Name %s", info_.joints[i].name.c_str());

    if (info_.joints[i].command_interfaces[0].name == hardware_interface::HW_IF_VELOCITY) {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_command_velocity_[i]));
        RCLCPP_INFO(rclcpp::get_logger("IsaacDriveHardware"), "Velocity: %s", info_.joints[i].name.c_str() );
        joint_names_velocity_.emplace_back(info_.joints[i].name);
        // counter_velocity++;
    } 
    // else {
    //   command_interfaces.emplace_back(hardware_interface::CommandInterface(
    //     info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_command_position_[counter_position]));
    //     RCLCPP_INFO(rclcpp::get_logger("IsaacDriveHardware"), "Position: %s", info_.joints[i].name.c_str() );
    //     joint_names_position_.emplace_back(info_.joints[i].name);
    //     counter_position++;
    // }
  }

  return command_interfaces;
}



CallbackReturn IsaacDriveHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("IsaacDriveHardware"), "Activating ...please wait...");

  // Set Default Values for State Interface Arrays
  for (auto i = 0u; i < hw_positions_.size(); i++)
  {
    hw_positions_[i] = NULL;
    hw_velocities_[i] = NULL;
    joint_names_map_[joint_names_[i]] = i + 1; // ADD 1 to differentiate null key
  }

  // Set Default Values for Command Interface Arrays
  for (auto i = 0u; i < hw_command_velocity_.size(); i++)
  {
    hw_command_velocity_[i] = 0;
    // hw_command_position_[i] = 0;
  }

  subscriber_is_active_ = true;

  RCLCPP_INFO(rclcpp::get_logger("IsaacDriveHardware"), "Successfully activated!");

  return CallbackReturn::SUCCESS;
}



CallbackReturn IsaacDriveHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("IsaacDriveHardware"), "Deactivating ...please wait...");
  subscriber_is_active_ = false;
  RCLCPP_INFO(rclcpp::get_logger("IsaacDriveHardware"), "Successfully deactivated!");

  return CallbackReturn::SUCCESS;
}


// ||                        ||
// \/ THE STUFF THAT MATTERS \/

hardware_interface::return_type IsaacDriveHardware::read()
{
  rclcpp::spin_some(node_);
  std::shared_ptr<sensor_msgs::msg::JointState> last_command_msg;
  received_joint_msg_ptr_.get(last_command_msg);

  if (last_command_msg == nullptr)
  {
    RCLCPP_WARN(rclcpp::get_logger("IsaacDriveHardware"), "Velocity message received was a nullptr.");
    return hardware_interface::return_type::ERROR;
  }

  auto names = last_command_msg->name;
  auto positions = last_command_msg->position;
  auto velocities = last_command_msg->velocity;
  
  for (auto i = 0u; i < names.size(); i++) {
    uint p = joint_names_map_[names[i]];
    if (p > 0) {
      hw_positions_[p - 1] = positions[i];
      hw_velocities_[p - 1] = velocities[i];
    }
  }
  
  return hardware_interface::return_type::OK;
}



hardware_interface::return_type swerve_hardware::IsaacDriveHardware::write()
{
  //RCLCPP_INFO(rclcpp::get_logger("IsaacDriveHardware"), "Velocity: %f", hw_command_velocity_[0]);

  // Publish Velocity
  if (realtime_isaac_publisher_->trylock()) {
    auto & realtime_isaac_command_ = realtime_isaac_publisher_->msg_;
    realtime_isaac_command_.header.stamp = node_->get_clock()->now();
    realtime_isaac_command_.name = joint_names_velocity_;
    realtime_isaac_command_.velocity = hw_command_velocity_;
    realtime_isaac_command_.position = empty_;
    realtime_isaac_publisher_->unlockAndPublish();
  }
  rclcpp::spin_some(node_);

  // Publish Position
  // if (realtime_isaac_publisher_->trylock()) {
  //   auto & realtime_isaac_command_ = realtime_isaac_publisher_->msg_;
  //   realtime_isaac_command_.header.stamp = node_->get_clock()->now();
  //   realtime_isaac_command_.name = joint_names_position_;
  //   realtime_isaac_command_.velocity = empty_;
  //   realtime_isaac_command_.position = hw_command_position_;
  //   realtime_isaac_publisher_->unlockAndPublish();
  // }
  // rclcpp::spin_some(node_);

  return hardware_interface::return_type::OK;
}

}  // namespace swerve_hardware



#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  swerve_hardware::IsaacDriveHardware, hardware_interface::SystemInterface)