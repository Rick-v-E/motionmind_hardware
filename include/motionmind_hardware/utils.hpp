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

#ifndef MOTIONMIND_HARDWARE_UTILS_HPP_
#define MOTIONMIND_HARDWARE_UTILS_HPP_

#include <string>
#include <vector>
#include <unordered_map>

#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"

bool checkChecksum(std::vector<uint8_t> result);
std::vector<uint8_t> createCommand(uint8_t command, uint8_t address, int value, int num_bytes);
hardware_interface::return_type getParamString(std::unordered_map<std::string, std::string> parameter_map, std::string parameter_name, std::string *variable_ptr);
hardware_interface::return_type getParamInt(std::unordered_map<std::string, std::string> parameter_map, std::string parameter_name, int *variable_ptr);
hardware_interface::return_type getParamBoolean(std::unordered_map<std::string, std::string> parameter_map, std::string parameter_name, bool *variable_ptr);

#endif
