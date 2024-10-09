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

#ifndef awo_CONTROL__DIFFBOT_SYSTEM_HPP_
#define awo_CONTROL__DIFFBOT_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "awo_control/visibility_control.h"


#include "awo_control/arduino_comms.hpp"
#include "awo_control/wheel.hpp"
#include "awo_control/camera_gimbal.hpp"

namespace awo_control
{
struct Config
{
  std::string Z_axis_name = "";
  std::string Y_axis_name = "";
  std::string X_axis_name = "";

  std::string left_wheel_name = "";
  std::string right_wheel_name = "";
  float loop_rate = 0.0;
  std::string device = "";
  int baud_rate = 0;
  int timeout_ms = 0;
  int enc_counts_per_rev = 0;
  int pid_p = 0;
  int pid_d = 0;
  int pid_i = 0;
  int pid_o = 0;
};
class awoDiffDriveHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(awoDiffDriveHardware);

  awo_CONTROL_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  awo_CONTROL_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  awo_CONTROL_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  awo_CONTROL_PUBLIC
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  awo_CONTROL_PUBLIC
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  awo_CONTROL_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  awo_CONTROL_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  awo_CONTROL_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  awo_CONTROL_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  ArduinoComms comms_;


  Config cfg_;

  Camera_joint z_axis_;
  Camera_joint y_axis_;
  Camera_joint x_axis_;
  Wheel wheel_l_;
  Wheel wheel_r_;
};

}  // namespace awo_control

#endif  // awo_CONTROL__DIFFBOT_SYSTEM_HPP_
