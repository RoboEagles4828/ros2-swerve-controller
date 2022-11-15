// Copyright 2020 PAL Robotics S.L.
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

/*
 * Author: Bence Magyar, Enrique Fernández, Manuel Meraz
 */

#ifndef SWERVE_CONTROLLER__SWERVE_CONTROLLER_HPP_
#define SWERVE_CONTROLLER__SWERVE_CONTROLLER_HPP_

#include <chrono>
#include <cmath>
#include <memory>
#include <queue>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "swerve_controller/visibility_control.h"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "hardware_interface/handle.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_box.h"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include <hardware_interface/loaned_command_interface.hpp>

namespace swerve_controller
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;


class Wheel {
  public:
    Wheel(std::reference_wrapper<hardware_interface::LoanedCommandInterface> velocity, std::string name);
    void set_velocity(double velocity);

  private:
    std::reference_wrapper<hardware_interface::LoanedCommandInterface> velocity_;
    std::string name;
};
class Axle {
  public:
    Axle(std::reference_wrapper<hardware_interface::LoanedCommandInterface> position, std::string name);
    void set_position(double position);

  private:
    std::reference_wrapper<hardware_interface::LoanedCommandInterface> position_;
    std::string name;


};

class SwerveController : public controller_interface::ControllerInterface
{
  using Twist = geometry_msgs::msg::TwistStamped;

public:
  SWERVE_CONTROLLER_PUBLIC
  SwerveController();

  SWERVE_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  SWERVE_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  SWERVE_CONTROLLER_PUBLIC
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  SWERVE_CONTROLLER_PUBLIC
  CallbackReturn on_init() override;

  SWERVE_CONTROLLER_PUBLIC
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  SWERVE_CONTROLLER_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  SWERVE_CONTROLLER_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  SWERVE_CONTROLLER_PUBLIC
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

  SWERVE_CONTROLLER_PUBLIC
  CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

  SWERVE_CONTROLLER_PUBLIC
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

protected:
  std::shared_ptr<Wheel> get_wheel(const std::string & wheel_name);
  std::shared_ptr<Axle> get_axle(const std::string & axle_name);
  std::shared_ptr<Wheel> front_left_handle_;
  std::shared_ptr<Wheel> front_right_handle_;
  std::shared_ptr<Wheel> rear_left_handle_;
  std::shared_ptr<Wheel> rear_right_handle_;
  std::shared_ptr<Axle> front_left_handle_2_;
  std::shared_ptr<Axle> front_right_handle_2_;
  std::shared_ptr<Axle> rear_left_handle_2_;
  std::shared_ptr<Axle> rear_right_handle_2_;
  std::string front_left_wheel_joint_name_;
  std::string front_right_wheel_joint_name_;
  std::string rear_left_wheel_joint_name_;
  std::string rear_right_wheel_joint_name_;
  std::string front_left_axle_joint_name_;
  std::string front_right_axle_joint_name_;
  std::string rear_left_axle_joint_name_;
  std::string rear_right_axle_joint_name_;
  struct WheelParams
  {
    double x_offset = 0.0; // Chassis Center to Axle Center
    double y_offset = 0.0; // Axle Center to Wheel Center
    double radius = 0.0;   // Assumed to be the same for all wheels
  } wheel_params_;

  // Timeout to consider cmd_vel commands old
  std::chrono::milliseconds cmd_vel_timeout_{500};
  rclcpp::Time previous_update_timestamp_{0};

  // Topic Subscription
  bool subscriber_is_active_ = false;
  rclcpp::Subscription<Twist>::SharedPtr velocity_command_subscriber_ = nullptr;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
    velocity_command_unstamped_subscriber_ = nullptr;

  realtime_tools::RealtimeBox<std::shared_ptr<Twist>> received_velocity_msg_ptr_{nullptr};

  bool is_halted = false;
  bool use_stamped_vel_ = true;

  bool reset();
  void halt();
};
}  // namespace swerve_controllerS
#endif  // Swerve_CONTROLLER__SWERVE_CONTROLLER_HPP_
