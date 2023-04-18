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

#include "motionmind_hardware/utils.hpp"

using hardware_interface::return_type;

bool checkChecksum(std::vector<uint8_t> result)
{
    uint8_t checksum = result.back();
    int sum = 0;

    for (size_t i = 0; i < result.size() - 1; ++i)
    {
        sum += static_cast<int>(result[i]);
    }

    return ((sum & 0xff) == checksum);
}

std::vector<uint8_t> createCommand(uint8_t command, uint8_t address, int value, int num_bytes = 4)
{
    std::vector<uint8_t> raw_cmd;
    int checksum = 0;

    raw_cmd.push_back(command);
    raw_cmd.push_back(address);
    checksum += command + address;

    for (int i = 0; i < num_bytes; ++i)
    {
        uint8_t byte = (value >> i * 8) & 0xff;
        raw_cmd.push_back(byte);
        checksum += byte;
    }

    raw_cmd.push_back(checksum & 0xff);
    return raw_cmd;
}

return_type getParamString(std::unordered_map<std::string, std::string> parameter_map, std::string parameter_name, std::string *variable_ptr)
{
    if (parameter_map.find(parameter_name) == parameter_map.end())
    {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("MotionmindHardware"), "Cannot find parameter '" << parameter_name << "'!");
        return return_type::ERROR;
    }

    *variable_ptr = parameter_map.at(parameter_name);
    return return_type::OK;
}

return_type getParamInt(std::unordered_map<std::string, std::string> parameter_map, std::string parameter_name, int *variable_ptr)
{
    if (parameter_map.find(parameter_name) == parameter_map.end())
    {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("MotionmindHardware"), "Cannot find parameter '" << parameter_name << "'!");
        return return_type::ERROR;
    }

    int value = std::stoi(parameter_map.at(parameter_name));
    *variable_ptr = value;
    return return_type::OK;
}

return_type getParamBoolean(std::unordered_map<std::string, std::string> parameter_map, std::string parameter_name, bool *variable_ptr)
{
    bool value = (parameter_map.find(parameter_name) != parameter_map.end() && parameter_map.at(parameter_name) == "true");
    *variable_ptr = value;

    return return_type::OK;
}
