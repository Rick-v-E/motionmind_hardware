/*
Copyright (c) 2023 Rick-v-E.

This file is part of the ROS2 Motionmind Hardware Interface
(see https//github.com/Rick-v-E/motionmind_hardware).

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef MOTIONMIND_HARDWARE__MOTIONMIND_HARDWARE_HPP_
#define MOTIONMIND_HARDWARE__MOTIONMIND_HARDWARE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <bitset>
#include <chrono>
#include <functional>

#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "serial/serial.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"

#include "motionmind_hardware/motionmind_definitions.hpp"
#include "motionmind_hardware/utils.hpp"
#include "motionmind_hardware/visibility_control.h"

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

namespace motionmind_hardware
{

  constexpr char HW_IF_NEGLIMIT[] = "neglimit";
  constexpr char HW_IF_POSLIMIT[] = "poslimit";
  constexpr char HW_IF_BRAKE[] = "brake";
  constexpr char HW_IF_INDEX[] = "index";
  constexpr char HW_IF_BADRC[] = "badrc";
  constexpr char HW_IF_VNLIMIT[] = "vnlimit";
  constexpr char HW_IF_VPLIMIT[] = "vplimit";
  constexpr char HW_IF_CURRENTLIMIT[] = "currentlimit";
  constexpr char HW_IF_PWMLIMIT[] = "pwmlimit";
  constexpr char HW_IF_INPOSITION[] = "inposition";
  constexpr char HW_IF_TEMPFAULT[] = "tempfault";

  constexpr uint8_t POSITION_MODE = 5;
  constexpr uint8_t VELOCITY_MODE = 4;

  struct CommandValue
  {
    double position{0.0};
    double velocity{0.0};
  };

  struct StateValue
  {
    double position{0.0};
    double velocity{0.0};
    double neglimit{0.0};
    double poslimit{0.0};
    double brake{0.0};
    double index{0.0};
    double badrc{0.0};
    double vnlimit{0.0};
    double vplimit{0.0};
    double currentlimit{0.0};
    double pwmlimit{0.0};
    double inposition{0.0};
    double tempfault{0.0};
  };

  struct JointInfo
  {
    std::string name;
    std::string command_interface_name;
    uint8_t address;
    int home;
    int min;
    int max;
    int cpr;
    int gear_ratio;
    std::string encoder_type;
  };

  struct Joint
  {
    StateValue state{};
    CommandValue command{};
    JointInfo joint_info;
  };

  class MotionmindHardware : public hardware_interface::SystemInterface
  {
  public:
    RCLCPP_SHARED_PTR_DEFINITIONS(MotionmindHardware)

    MOTIONMIND_HARDWARE_PUBLIC
    CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;

    MOTIONMIND_HARDWARE_PUBLIC
    CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;

    MOTIONMIND_HARDWARE_PUBLIC
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state) override;

    MOTIONMIND_HARDWARE_PUBLIC
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    MOTIONMIND_HARDWARE_PUBLIC
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    MOTIONMIND_HARDWARE_PUBLIC
    return_type perform_command_mode_switch(const std::vector<std::string> &start_interfaces, const std::vector<std::string> &stop_interfaces) override;

    MOTIONMIND_HARDWARE_PUBLIC
    CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

    MOTIONMIND_HARDWARE_PUBLIC
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

    MOTIONMIND_HARDWARE_PUBLIC
    return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    MOTIONMIND_HARDWARE_PUBLIC
    return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

  private:
    return_type stop();
    return_type checkMode(uint8_t address, uint8_t expected_mode);
    return_type reset_command();
    return_type readStatusPosition(uint8_t address, int &position, MotionmindStatus &status);
    return_type readStatusVelocity(uint8_t address, int &velocity, MotionmindStatus &status);
    return_type writePosition(uint8_t address, int position);
    return_type writeVelocity(uint8_t address, int position);
    return_type loadPositionParameters(int joint_idx);
    return_type loadVelocityParameters(int joint_idx);

    std::vector<Joint> joints_;
    std::string port_;
    int baudrate_;
    int timeout_;
    serial::Serial serial_connection_;
    bool use_dummy_ = false;
  };

} // namespace motionmind_hardware

#endif