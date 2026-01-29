// Copyright (c) 2026, valerio
// Copyright (c) 2026, b»robotized group (template)
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

#pragma once

#define MY_COOL_ROBOT_CONTROL_PACKAGE__MY_COOL_ROBOT_HARDWARE_HPP_
#define MY_COOL_ROBOT_CONTROL_PACKAGE__MY_COOL_ROBOT_HARDWARE_HPP_

#include <string>
#include <vector>
#include <unordered_map>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "SCServo.h" // fourni par FTServo_Linux

namespace my_cool_robot_control_package
{
class MyCoolRobotHardware : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // ===== ROS2 control buffers =====
  std::vector<double> hw_states_pos_; // rad
  std::vector<double> hw_states_vel_; // rad/s (optionnel)
  std::vector<double> hw_commands_pos_; // rad (position command)

  // ===== Feetech SDK =====
  SMS_STS bus_;
  bool connected_{false};

  // ===== Config =====
  std::string device_{"/dev/ttyACM0"};
  int baud_{1000000};

  std::vector<int> motor_ids_; // size = nb joints
  std::vector<int> offset_ticks_; // size = nb joints
  std::vector<int> sign_; // +1 ou -1 par joint

  // helpers
  static constexpr double TICKS_PER_REV = 4096.0;
  static constexpr double TWO_PI = 6.283185307179586;

  double ticks_to_rad(int ticks, int offset, int sign) const;
  int rad_to_ticks(double rad, int offset, int sign) const;
};

}  // namespace my_cool_robot_control_package
