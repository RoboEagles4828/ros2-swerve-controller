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

#include <memory>
#include <queue>
#include <string>
#include <utility>
#include <vector>
#include <cmath>

#include "swerve_controller/swerve_controller.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/logging.hpp"

namespace
{
  constexpr auto DEFAULT_COMMAND_TOPIC = "~/cmd_vel";
  constexpr auto DEFAULT_COMMAND_UNSTAMPED_TOPIC = "~/cmd_vel_unstamped";
  constexpr auto DEFAULT_COMMAND_OUT_TOPIC = "~/cmd_vel_out";
} // namespace

namespace swerve_controller
{
  using namespace std::chrono_literals;
  using controller_interface::interface_configuration_type;
  using controller_interface::InterfaceConfiguration;
  using hardware_interface::HW_IF_POSITION;
  using hardware_interface::HW_IF_VELOCITY;
  using lifecycle_msgs::msg::State;

  Wheel::Wheel(std::reference_wrapper<hardware_interface::LoanedCommandInterface> velocity, std::string name) : velocity_(velocity), name(std::move(name)) {}

  void Wheel::set_velocity(double velocity)
  {
    velocity_.get().set_value(velocity);
  }
  Axle::Axle(std::reference_wrapper<hardware_interface::LoanedCommandInterface> velocity, std::reference_wrapper<const hardware_interface::LoanedStateInterface> position, std::string name) : velocity_(velocity), position_(position), name(std::move(name)) {}

  void Axle::set_velocity(double velocity)
  {
    velocity_.get().set_value(velocity);
  }
  double Axle::get_position(void)
  {
    // temporary
    return position_.get().get_value();
  }
  SwerveController::SwerveController() : controller_interface::ControllerInterface() {}

  CallbackReturn SwerveController::on_init()
  {
    try
    {
      // with the lifecycle node being initialized, we can declare parameters
      auto_declare<std::string>("front_left_wheel_joint", front_left_wheel_joint_name_);
      auto_declare<std::string>("front_right_wheel_joint", front_right_wheel_joint_name_);
      auto_declare<std::string>("rear_left_wheel_joint", rear_left_wheel_joint_name_);
      auto_declare<std::string>("rear_right_wheel_joint", rear_right_wheel_joint_name_);

      auto_declare<std::string>("front_left_axle_joint", front_left_axle_joint_name_);
      auto_declare<std::string>("front_right_axle_joint", front_right_axle_joint_name_);
      auto_declare<std::string>("rear_left_axle_joint", rear_left_axle_joint_name_);
      auto_declare<std::string>("rear_right_axle_joint", rear_right_axle_joint_name_);

      auto_declare<double>("chassis_length", wheel_params_.x_offset);
      auto_declare<double>("chassis_width", wheel_params_.y_offset);
      auto_declare<double>("wheel_radius", wheel_params_.radius);

      auto_declare<double>("cmd_vel_timeout", cmd_vel_timeout_.count() / 1000.0);
      auto_declare<bool>("use_stamped_vel", use_stamped_vel_);
    }
    catch (const std::exception &e)
    {
      fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
      return CallbackReturn::ERROR;
    }

    return CallbackReturn::SUCCESS;
  }

  InterfaceConfiguration SwerveController::command_interface_configuration() const
  {
    std::vector<std::string> conf_names;
    conf_names.push_back(front_left_wheel_joint_name_ + "/" + HW_IF_VELOCITY);
    conf_names.push_back(front_right_wheel_joint_name_ + "/" + HW_IF_VELOCITY);
    conf_names.push_back(rear_left_wheel_joint_name_ + "/" + HW_IF_VELOCITY);
    conf_names.push_back(rear_right_wheel_joint_name_ + "/" + HW_IF_VELOCITY);
    conf_names.push_back(front_left_axle_joint_name_ + "/" + HW_IF_VELOCITY);
    conf_names.push_back(front_right_axle_joint_name_ + "/" + HW_IF_VELOCITY);
    conf_names.push_back(rear_left_axle_joint_name_ + "/" + HW_IF_VELOCITY);
    conf_names.push_back(rear_right_axle_joint_name_ + "/" + HW_IF_VELOCITY);
    return {interface_configuration_type::INDIVIDUAL, conf_names};
  }

  InterfaceConfiguration SwerveController::state_interface_configuration() const
  {
    std::vector<std::string> conf_names;
    conf_names.push_back(front_left_axle_joint_name_ + "/" + HW_IF_POSITION);
    conf_names.push_back(front_right_axle_joint_name_ + "/" + HW_IF_POSITION);
    conf_names.push_back(rear_left_axle_joint_name_ + "/" + HW_IF_POSITION);
    conf_names.push_back(rear_right_axle_joint_name_ + "/" + HW_IF_POSITION);
    return {interface_configuration_type::INDIVIDUAL, conf_names};
  }

  controller_interface::return_type SwerveController::update(
      const rclcpp::Time &time, const rclcpp::Duration & /*period*/)
  {
    auto logger = node_->get_logger();
    if (get_state().id() == State::PRIMARY_STATE_INACTIVE)
    {
      if (!is_halted)
      {
        halt();
        is_halted = true;
      }
      return controller_interface::return_type::OK;
    }

    const auto current_time = time;

    std::shared_ptr<Twist> last_command_msg;
    received_velocity_msg_ptr_.get(last_command_msg);

    if (last_command_msg == nullptr)
    {
      RCLCPP_WARN(logger, "Velocity message received was a nullptr.");
      return controller_interface::return_type::ERROR;
    }

    const auto age_of_last_command = current_time - last_command_msg->header.stamp;
    // Brake if cmd_vel has timeout, override the stored command
    if (age_of_last_command > cmd_vel_timeout_)
    {
      last_command_msg->twist.linear.x = 0.0;
      last_command_msg->twist.angular.z = 0.0;
    }

    Twist command = *last_command_msg;
    double &linear_x_cmd = command.twist.linear.x;
    double &linear_y_cmd = command.twist.linear.y;
    double &angular_cmd = command.twist.angular.z;
    if (abs(linear_x_cmd) < 0.5 && abs(linear_y_cmd) < 0.5 && abs(angular_cmd) < 0.5)
    {
      RCLCPP_INFO(logger, "yes");
      front_left_handle_->set_velocity(0.0);
      front_right_handle_->set_velocity(0.0);
      rear_left_handle_->set_velocity(0.0);
      rear_right_handle_->set_velocity(0.0);
      front_left_handle_2_->set_velocity(0.0);
      front_right_handle_2_->set_velocity(0.0);
      rear_left_handle_2_->set_velocity(0.0);
      rear_right_handle_2_->set_velocity(0.0);
      const auto update_dt = current_time - previous_update_timestamp_;
      previous_update_timestamp_ = current_time;
      return controller_interface::return_type::OK;
    }
    else
    {
      RCLCPP_INFO(logger, "NO");
      double x_offset = wheel_params_.x_offset;
      double radius = wheel_params_.radius;

      // Compute Wheel Velocities and Positions
      const double a = linear_x_cmd - angular_cmd * x_offset / 2;

      const double b = linear_x_cmd + angular_cmd * x_offset / 2;

      const double c = linear_y_cmd - angular_cmd * x_offset / 2;

      const double d = linear_y_cmd + angular_cmd * x_offset / 2;

      // get current wheel positions
      const double front_left_current_pos = front_left_handle_2_->get_position();
      const double front_right_current_pos = front_right_handle_2_->get_position();
      const double rear_left_current_pos = rear_left_handle_2_->get_position();
      const double rear_right_current_pos = rear_right_handle_2_->get_position();
      // RCLCPP_INFO(rclcpp::get_logger("update"), "%f", pos1);

      double front_left_velocity = (sqrt(pow(b, 2) + pow(d, 2))) * (1 / (radius * M_PI));
      double front_right_velocity = (sqrt(pow(b, 2) + pow(c, 2))) * (1 / (radius * M_PI));
      double rear_left_velocity = (sqrt(pow(a, 2) + pow(d, 2))) * (1 / (radius * M_PI));
      double rear_right_velocity = (sqrt(pow(a, 2) + pow(c, 2))) * (1 / (radius * M_PI));

      double front_left_position = atan2(b, d);
      double front_right_position = atan2(b, c);
      double rear_left_position = atan2(a, d);
      double rear_right_position = atan2(a, c);

      // optimization

      // front_left
      // RCLCPP_INFO(logger, "front_left_current_pos: %f, front_left_position %f", front_left_current_pos, front_left_position);
      while (abs(front_left_current_pos - front_left_position) > M_PI / 2)
      {
        if (front_left_position > front_left_current_pos)
        {
          front_left_position -= M_PI;
        }
        else
        {
          front_left_position += M_PI;
        }
        RCLCPP_INFO(logger, "front_left_current_pos");

        // // convert to degrees
        // front_left_position = ((180 / M_PI) * front_left_position);
        // // keep it in -90 to 90 scope
        // front_left_position = ((int)(front_left_position) + 90) % 180 - 90;
        // // convert back to radians
        // if (abs(front_left_position) >= 0.01)
        // {
        //   front_left_position = (M_PI / 180) * front_left_position;
        // }

        front_left_velocity *= -1;
      }
      // front_right
      while (abs(front_right_current_pos - front_right_position) > M_PI / 2)
      {
        if (front_right_position > front_right_current_pos)
        {
          front_right_position -= M_PI;
        }
        else
        {
          front_right_position += M_PI;
        }
        RCLCPP_INFO(logger, "front_right_current_pos");
        // // convert to degrees
        // front_right_position = ((180 / M_PI) * front_right_position);
        // // keep it in -90 to 90 scope
        // front_right_position = ((int)(front_right_position) + 90) % 180 - 90;
        // // convert back to radians
        // if (abs(front_right_position) >= 0.01)
        // {
        //   front_right_position = (M_PI / 180) * front_right_position;
        // }

        front_right_velocity *= -1;
      }
      // rear_left
      while (abs(rear_left_current_pos - rear_left_position) > M_PI / 2)
      {
        if (rear_left_position > rear_left_current_pos)
        {
          rear_left_position -= M_PI;
        }
        else
        {
          rear_left_position += M_PI;
        }
        RCLCPP_INFO(logger, "rear_left_current_pos");

        // //convert to degrees
        // rear_left_position = ((180 / M_PI) * rear_left_position);
        // //keep it in -90 to 90 scope
        // rear_left_position = ((int)(rear_left_position) + 90) % 180 - 90;
        // //convert back to radians
        // if(abs(rear_left_position)>=0.01){
        //   rear_left_position = (M_PI / 180) * rear_left_position;
        // }

        rear_left_velocity *= -1;
      }
      // rear right
      while (abs(rear_right_current_pos - rear_right_position) > M_PI / 2)
      {
        if (rear_right_position > rear_right_current_pos)
        {
          rear_right_position -= M_PI;
        }
        else
        {
          rear_right_position += M_PI;
        }
        RCLCPP_INFO(logger, "rear_right_current_pos");
        // //convert to degrees
        // rear_right_position = ((180 / M_PI) * rear_right_position);
        // //keep it in -90 to 90 scope
        // rear_right_position = ((int)(rear_right_position) + 90) % 180 - 90;
        // //convert back to radians
        // if(abs(rear_right_position)>=0.01){
        //   rear_right_position = (M_PI / 180) * rear_right_position;
        rear_right_velocity *= -1;
      }
      // RCLCPP_INFO(logger, "front_left_position: %f", front_left_position);
      // RCLCPP_INFO(logger, "front_left_current_position: %f", front_left_current_pos);
      // Set Wheel Velocities
      front_left_handle_->set_velocity(front_left_velocity);
      front_right_handle_->set_velocity(front_right_velocity);
      rear_left_handle_->set_velocity(rear_left_velocity);
      rear_right_handle_->set_velocity(rear_right_velocity);

      // Set Wheel Positions
      // remmeber to comment this back in!
      // Has a 1 degree tolerance. Turns clockwise if less than, counter clockwise if greater than
      float turningspeed = 5.0;
      if (front_left_current_pos > front_left_position + (M_PI / 90) || front_left_current_pos < front_left_position - (M_PI / 90))
      {
        float setspeed = abs((front_left_position - front_left_current_pos)) / (M_PI / 18);
        if (setspeed > turningspeed)
        {
          setspeed = turningspeed;
        }
        if (front_left_position > front_left_current_pos)
        {
          front_left_handle_2_->set_velocity(setspeed);
        }
        else
        {
          front_left_handle_2_->set_velocity(-1 * setspeed);
        }
      }
      else
      {
        front_left_handle_2_->set_velocity(0.0);
      }
      if (front_right_current_pos > front_right_position + (M_PI / 90) || front_right_current_pos < front_right_position - (M_PI / 90))
      {
        float setspeed = abs((front_right_position - front_right_current_pos)) / (M_PI / 18);
        if (setspeed > turningspeed)
        {
          setspeed = turningspeed;
        }
        if (front_right_position > front_right_current_pos)
        {
          front_right_handle_2_->set_velocity(setspeed);
        }
        else
        {
          front_right_handle_2_->set_velocity(-1 * setspeed);
        }
      }
      else
      {
        front_left_handle_2_->set_velocity(0.0);
      }
      if (rear_left_current_pos > rear_left_position + (M_PI / 90) || rear_left_current_pos < rear_left_position - (M_PI / 90))
      {
        float setspeed = (abs(rear_left_position - rear_left_current_pos)) / (M_PI / 18);
        if (setspeed > turningspeed)
        {
          setspeed = turningspeed;
        }
        if (rear_left_position > rear_left_current_pos)
        {
          rear_left_handle_2_->set_velocity(setspeed);
        }
        else
        {
          rear_left_handle_2_->set_velocity(-1 * setspeed);
        }
      }
      else
      {
        front_left_handle_2_->set_velocity(0.0);
      }
      if (rear_right_current_pos > rear_right_position + (M_PI / 90) || rear_right_current_pos < rear_right_position - (M_PI / 90))
      {
        float setspeed = (abs(rear_right_position - rear_right_current_pos)) / (M_PI / 18);
        if (setspeed > turningspeed)
        {
          setspeed = turningspeed;
        }
        if (rear_right_position > rear_right_current_pos)
        {
          rear_right_handle_2_->set_velocity(setspeed);
        }
        else
        {
          rear_right_handle_2_->set_velocity(-1 * setspeed);
        }
      }
      else
      {
        front_left_handle_2_->set_velocity(0.0);
      }

      // remmeber to comment this back in!

      // test TEMPORARY!
      //  front_left_handle_2_->set_velocity(15.0);
      //  front_right_handle_2_->set_velocity(5.0);
      //  rear_left_handle_2_->set_velocity(5.0);
      //  rear_right_handle_2_->set_velocity(5.0);
      // test TEMPORARY!

      // Time update
      const auto update_dt = current_time - previous_update_timestamp_;
      previous_update_timestamp_ = current_time;

      return controller_interface::return_type::OK;
    }
  }

  CallbackReturn SwerveController::on_configure(const rclcpp_lifecycle::State &)
  {
    auto logger = node_->get_logger();

    // Get Parameters
    front_left_wheel_joint_name_ = node_->get_parameter("front_left_wheel_joint").as_string();
    front_right_wheel_joint_name_ = node_->get_parameter("front_right_wheel_joint").as_string();
    rear_left_wheel_joint_name_ = node_->get_parameter("rear_left_wheel_joint").as_string();
    rear_right_wheel_joint_name_ = node_->get_parameter("rear_right_wheel_joint").as_string();

    front_left_axle_joint_name_ = node_->get_parameter("front_left_axle_joint").as_string();
    front_right_axle_joint_name_ = node_->get_parameter("front_right_axle_joint").as_string();
    rear_left_axle_joint_name_ = node_->get_parameter("rear_left_axle_joint").as_string();
    rear_right_axle_joint_name_ = node_->get_parameter("rear_right_axle_joint").as_string();

    if (front_left_wheel_joint_name_.empty())
    {
      RCLCPP_ERROR(logger, "front_left_wheel_joint_name is not set");
      return CallbackReturn::ERROR;
    }
    if (front_right_wheel_joint_name_.empty())
    {
      RCLCPP_ERROR(logger, "front_right_wheel_joint_name is not set");
      return CallbackReturn::ERROR;
    }
    if (rear_left_wheel_joint_name_.empty())
    {
      RCLCPP_ERROR(logger, "rear_left_wheel_joint_name is not set");
      return CallbackReturn::ERROR;
    }
    if (rear_right_wheel_joint_name_.empty())
    {
      RCLCPP_ERROR(logger, "rear_right_wheel_joint_name is not set");
      return CallbackReturn::ERROR;
    }

    if (front_left_axle_joint_name_.empty())
    {
      RCLCPP_ERROR(logger, "front_left_axle_joint_name is not set");
      return CallbackReturn::ERROR;
    }
    if (front_right_axle_joint_name_.empty())
    {
      RCLCPP_ERROR(logger, "front_right_axle_joint_name is not set");
      return CallbackReturn::ERROR;
    }
    if (rear_left_axle_joint_name_.empty())
    {
      RCLCPP_ERROR(logger, "rear_left_axle_joint_name is not set");
      return CallbackReturn::ERROR;
    }
    if (rear_right_axle_joint_name_.empty())
    {
      RCLCPP_ERROR(logger, "rear_right_axle_joint_name is not set");
      return CallbackReturn::ERROR;
    }

    wheel_params_.x_offset = node_->get_parameter("chassis_length").as_double();
    wheel_params_.y_offset = node_->get_parameter("chassis_width").as_double();
    wheel_params_.radius = node_->get_parameter("wheel_radius").as_double();

    cmd_vel_timeout_ = std::chrono::milliseconds{
        static_cast<int>(node_->get_parameter("cmd_vel_timeout").as_double() * 1000.0)};
    use_stamped_vel_ = node_->get_parameter("use_stamped_vel").as_bool();

    // Run reset to make sure everything is initialized correctly
    if (!reset())
    {
      return CallbackReturn::ERROR;
    }

    const Twist empty_twist;
    received_velocity_msg_ptr_.set(std::make_shared<Twist>(empty_twist));

    // initialize command subscriber
    if (use_stamped_vel_)
    {
      velocity_command_subscriber_ = node_->create_subscription<Twist>(
          DEFAULT_COMMAND_TOPIC, rclcpp::SystemDefaultsQoS(),
          [this](const std::shared_ptr<Twist> msg) -> void
          {
            if (!subscriber_is_active_)
            {
              RCLCPP_WARN(node_->get_logger(), "Can't accept new commands. subscriber is inactive");
              return;
            }
            if ((msg->header.stamp.sec == 0) && (msg->header.stamp.nanosec == 0))
            {
              RCLCPP_WARN_ONCE(
                  node_->get_logger(),
                  "Received TwistStamped with zero timestamp, setting it to current "
                  "time, this message will only be shown once");
              msg->header.stamp = node_->get_clock()->now();
            }
            received_velocity_msg_ptr_.set(std::move(msg));
          });
    }
    else
    {
      velocity_command_unstamped_subscriber_ = node_->create_subscription<geometry_msgs::msg::Twist>(
          DEFAULT_COMMAND_UNSTAMPED_TOPIC, rclcpp::SystemDefaultsQoS(),
          [this](const std::shared_ptr<geometry_msgs::msg::Twist> msg) -> void
          {
            if (!subscriber_is_active_)
            {
              RCLCPP_WARN(node_->get_logger(), "Can't accept new commands. subscriber is inactive");
              return;
            }

            // Write fake header in the stored stamped command
            std::shared_ptr<Twist> twist_stamped;
            received_velocity_msg_ptr_.get(twist_stamped);
            twist_stamped->twist = *msg;
            twist_stamped->header.stamp = node_->get_clock()->now();
          });
    }

    previous_update_timestamp_ = node_->get_clock()->now();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn SwerveController::on_activate(const rclcpp_lifecycle::State &)
  {
    front_left_handle_ = get_wheel(front_left_wheel_joint_name_);
    front_right_handle_ = get_wheel(front_right_wheel_joint_name_);
    rear_left_handle_ = get_wheel(rear_left_wheel_joint_name_);
    rear_right_handle_ = get_wheel(rear_right_wheel_joint_name_);
    front_left_handle_2_ = get_axle(front_left_axle_joint_name_);
    front_right_handle_2_ = get_axle(front_right_axle_joint_name_);
    rear_left_handle_2_ = get_axle(rear_left_axle_joint_name_);
    rear_right_handle_2_ = get_axle(rear_right_axle_joint_name_);

    if (!front_left_handle_ || !front_right_handle_ || !rear_left_handle_ || !rear_right_handle_ || !front_left_handle_2_ || !front_right_handle_2_ || !rear_left_handle_2_ || !rear_right_handle_2_)
    {
      return CallbackReturn::ERROR;
    }

    is_halted = false;
    subscriber_is_active_ = true;

    RCLCPP_DEBUG(node_->get_logger(), "Subscriber and publisher are now active.");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn SwerveController::on_deactivate(const rclcpp_lifecycle::State &)
  {
    subscriber_is_active_ = false;
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn SwerveController::on_cleanup(const rclcpp_lifecycle::State &)
  {
    if (!reset())
    {
      return CallbackReturn::ERROR;
    }

    received_velocity_msg_ptr_.set(std::make_shared<Twist>());
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn SwerveController::on_error(const rclcpp_lifecycle::State &)
  {
    if (!reset())
    {
      return CallbackReturn::ERROR;
    }
    return CallbackReturn::SUCCESS;
  }

  bool SwerveController::reset()
  {
    subscriber_is_active_ = false;
    velocity_command_subscriber_.reset();
    velocity_command_unstamped_subscriber_.reset();

    received_velocity_msg_ptr_.set(nullptr);
    is_halted = false;
    return true;
  }

  CallbackReturn SwerveController::on_shutdown(const rclcpp_lifecycle::State &)
  {
    return CallbackReturn::SUCCESS;
  }

  void SwerveController::halt()
  {
    front_left_handle_->set_velocity(0.0);
    front_right_handle_->set_velocity(0.0);
    rear_left_handle_->set_velocity(0.0);
    rear_right_handle_->set_velocity(0.0);
    auto logger = node_->get_logger();
    RCLCPP_WARN(logger, "-----HALT CALLED : STOPPING ALL MOTORS-----");
  }

  std::shared_ptr<Wheel> SwerveController::get_wheel(const std::string &wheel_name)
  {
    auto logger = node_->get_logger();
    if (wheel_name.empty())
    {
      RCLCPP_ERROR(logger, "Wheel joint name not given. Make sure all joints are specified.");
      return nullptr;
    }

    // Get Command Handle for joint
    const auto command_handle = std::find_if(
        command_interfaces_.begin(), command_interfaces_.end(),
        [&wheel_name](const auto &interface)
        {
          return interface.get_name() == wheel_name &&
                 interface.get_interface_name() == HW_IF_VELOCITY;
        });

    if (command_handle == command_interfaces_.end())
    {
      RCLCPP_ERROR(logger, "Unable to obtain joint command handle for %s", wheel_name.c_str());
      return nullptr;
    }
    return std::make_shared<Wheel>(std::ref(*command_handle), wheel_name);
  }
  std::shared_ptr<Axle> SwerveController::get_axle(const std::string &axle_name)
  {
    auto logger = node_->get_logger();
    if (axle_name.empty())
    {
      RCLCPP_ERROR(logger, "Axle joint name not given. Make sure all joints are specified.");
      return nullptr;
    }

    // Get Command Handle for joint
    const auto command_handle = std::find_if(
        command_interfaces_.begin(), command_interfaces_.end(),
        [&axle_name](const auto &interface)
        {
          return interface.get_name() == axle_name &&
                 interface.get_interface_name() == HW_IF_VELOCITY;
        });
    const auto state_handle = std::find_if(
        state_interfaces_.cbegin(), state_interfaces_.cend(),
        [&axle_name](const auto &interface)
        {
          return interface.get_name() == axle_name &&
                 interface.get_interface_name() == HW_IF_POSITION;
        });

    if (command_handle == command_interfaces_.end())
    {
      RCLCPP_ERROR(logger, "Unable to obtain joint command handle for %s", axle_name.c_str());
      return nullptr;
    }
    if (state_handle == state_interfaces_.cend())
    {
      RCLCPP_ERROR(logger, "Unable to obtain joint state handle for %s", axle_name.c_str());
      return nullptr;
    }
    return std::make_shared<Axle>(std::ref(*command_handle), std::ref(*state_handle), axle_name);
  }
} // namespace swerve_controller

#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(
    swerve_controller::SwerveController, controller_interface::ControllerInterface)
