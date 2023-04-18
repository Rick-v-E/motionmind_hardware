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

#include "motionmind_hardware/motionmind_hardware.hpp"

namespace motionmind_hardware
{
    static const std::string HW_NAME = "MotionmindHardware";
    static const double PI2 = 2 * M_PI;

    CallbackReturn MotionmindHardware::on_init(const hardware_interface::HardwareInfo &info)
    {
        // Custom bind on_shutdown to stop motors when shutting down control mode.
        // TODO: remove once https://github.com/ros-controls/ros2_control/issues/472 is solved
        rclcpp::on_shutdown(std::bind(&MotionmindHardware::stop, this));

        if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
        {
            return CallbackReturn::ERROR;
        }

        joints_.resize(info_.joints.size(), Joint());

        for (size_t i = 0; i < info_.joints.size(); i++)
        {
            joints_[i].state.position = std::numeric_limits<double>::quiet_NaN();
            joints_[i].state.velocity = std::numeric_limits<double>::quiet_NaN();
            joints_[i].state.neglimit = std::numeric_limits<double>::quiet_NaN();
            joints_[i].state.poslimit = std::numeric_limits<double>::quiet_NaN();
            joints_[i].state.brake = std::numeric_limits<double>::quiet_NaN();
            joints_[i].state.index = std::numeric_limits<double>::quiet_NaN();
            joints_[i].state.badrc = std::numeric_limits<double>::quiet_NaN();
            joints_[i].state.vnlimit = std::numeric_limits<double>::quiet_NaN();
            joints_[i].state.vplimit = std::numeric_limits<double>::quiet_NaN();
            joints_[i].state.currentlimit = std::numeric_limits<double>::quiet_NaN();
            joints_[i].state.pwmlimit = std::numeric_limits<double>::quiet_NaN();
            joints_[i].state.inposition = std::numeric_limits<double>::quiet_NaN();
            joints_[i].state.tempfault = std::numeric_limits<double>::quiet_NaN();
            joints_[i].command.position = std::numeric_limits<double>::quiet_NaN();
            joints_[i].command.velocity = std::numeric_limits<double>::quiet_NaN();
            joints_[i].joint_info.name = info_.joints[i].name;

            int address;
            if (getParamInt(info_.joints[i].parameters, "address", &address) != return_type::OK)
            {
                return CallbackReturn::ERROR;
            }
            joints_[i].joint_info.address = address;
        }

        if (getParamString(info_.hardware_parameters, "port", &port_) != return_type::OK)
        {
            return CallbackReturn::ERROR;
        }

        if (getParamInt(info_.hardware_parameters, "baudrate", &baudrate_) != return_type::OK)
        {
            return CallbackReturn::ERROR;
        }

        if (getParamInt(info_.hardware_parameters, "timeout", &timeout_) != return_type::OK)
        {
            return CallbackReturn::ERROR;
        }

        if (getParamBoolean(info_.hardware_parameters, "use_dummy", &use_dummy_) != return_type::OK)
        {
            return CallbackReturn::ERROR;
        }

        // Define dummy mode to test hardware interface without hardware connected
        if (use_dummy_)
        {
            RCLCPP_INFO(rclcpp::get_logger(HW_NAME), "dummy mode");
            ;
        }

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn MotionmindHardware::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        if (use_dummy_)
        {
            return CallbackReturn::SUCCESS;
        }

        try
        {
            RCLCPP_INFO_STREAM(rclcpp::get_logger(HW_NAME), "Opening serial port " << port_ << " with baudrate " << baudrate_ << "...");

            // Create serial connection
            auto to = serial::Timeout::simpleTimeout(timeout_);
            serial_connection_.setPort(port_);
            serial_connection_.setBaudrate(baudrate_);
            serial_connection_.setTimeout(to);

            if (serial_connection_.isOpen())
            {
                serial_connection_.close();
            }

            serial_connection_.open();

            // Init command modes
            std::vector<std::string> new_interfaces, old_interfaces;
            for (auto joint : info_.joints)
            {
                for (auto interface : joint.command_interfaces)
                {
                    new_interfaces.push_back(joint.name + "/" + interface.name);
                }
            }
            perform_command_mode_switch(new_interfaces, old_interfaces);
        }
        catch (serial::IOException &e)
        {
            RCLCPP_FATAL_STREAM(rclcpp::get_logger(HW_NAME), "Unable to open serial connection : " << e.what());
            return CallbackReturn::ERROR;
        }
        catch (serial::SerialException &e)
        {
            RCLCPP_FATAL_STREAM(rclcpp::get_logger(HW_NAME), "Unable to open serial connection : " << e.what());
            return CallbackReturn::ERROR;
        }
        catch (serial::PortNotOpenedException &e)
        {
            RCLCPP_FATAL_STREAM(rclcpp::get_logger(HW_NAME), "Unable to open serial connection : " << e.what());
            return CallbackReturn::ERROR;
        }

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn MotionmindHardware::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        if (use_dummy_)
        {
            return CallbackReturn::SUCCESS;
        }

        if (serial_connection_.isOpen())
        {
            serial_connection_.close();
        }

        return CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> MotionmindHardware::export_state_interfaces()
    {
        RCLCPP_DEBUG(rclcpp::get_logger(HW_NAME), "export_state_interfaces");
        std::vector<hardware_interface::StateInterface> state_interfaces;
        for (size_t i = 0; i < joints_.size(); i++)
        {
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joints_[i].state.position));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joints_[i].state.velocity));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, HW_IF_NEGLIMIT, &joints_[i].state.neglimit));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, HW_IF_POSLIMIT, &joints_[i].state.poslimit));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, HW_IF_BRAKE, &joints_[i].state.brake));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, HW_IF_INDEX, &joints_[i].state.index));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, HW_IF_BADRC, &joints_[i].state.badrc));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, HW_IF_VNLIMIT, &joints_[i].state.vnlimit));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, HW_IF_VPLIMIT, &joints_[i].state.vplimit));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, HW_IF_CURRENTLIMIT, &joints_[i].state.currentlimit));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, HW_IF_PWMLIMIT, &joints_[i].state.pwmlimit));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, HW_IF_INPOSITION, &joints_[i].state.inposition));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, HW_IF_TEMPFAULT, &joints_[i].state.tempfault));
        }

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> MotionmindHardware::export_command_interfaces()
    {
        RCLCPP_DEBUG(rclcpp::get_logger(HW_NAME), "export_command_interfaces");
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        for (size_t i = 0; i < joints_.size(); i++)
        {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joints_[i].command.position));
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joints_[i].command.velocity));
        }
        return command_interfaces;
    }

    return_type MotionmindHardware::perform_command_mode_switch(const std::vector<std::string> &start_interfaces, const std::vector<std::string> & /*stop_interfaces*/)
    {
        if (!use_dummy_ && !serial_connection_.isOpen())
        {
            RCLCPP_ERROR(rclcpp::get_logger(HW_NAME), "Could not connect to motor controllers!");
            return return_type::ERROR;
        }

        for (size_t i = 0; i < joints_.size(); i++)
        {
            for (auto interface : start_interfaces)
            {
                if (interface.find(joints_[i].joint_info.name) != std::string::npos)
                {
                    if (interface.find(hardware_interface::HW_IF_POSITION) != std::string::npos)
                    {
                        if (!use_dummy_ && checkMode(joints_[i].joint_info.address, motionmind_hardware::POSITION_MODE) != return_type::OK)
                        {
                            RCLCPP_ERROR_STREAM(rclcpp::get_logger(HW_NAME), "Motor controller '" << joints_[i].joint_info.name << "' is not in analog PID mode!");
                            return return_type::ERROR;
                        }

                        if (loadPositionParameters(i) != return_type::OK)
                        {
                            RCLCPP_ERROR_STREAM(rclcpp::get_logger(HW_NAME), "Could not load position parameters for motor controller '" << joints_[i].joint_info.name << "'!");
                            return return_type::ERROR;
                        }

                        joints_[i].joint_info.command_interface_name = hardware_interface::HW_IF_POSITION;
                    }
                    else if (interface.find(hardware_interface::HW_IF_VELOCITY) != std::string::npos)
                    {
                        if (!use_dummy_ && checkMode(joints_[i].joint_info.address, motionmind_hardware::VELOCITY_MODE) != return_type::OK)
                        {
                            RCLCPP_ERROR_STREAM(rclcpp::get_logger(HW_NAME), "Motor controller '" << joints_[i].joint_info.name << "' is not in serial PID mode!");
                            return return_type::ERROR;
                        }

                        if (loadVelocityParameters(i) != return_type::OK)
                        {
                            RCLCPP_ERROR_STREAM(rclcpp::get_logger(HW_NAME), "Could not load velocity parameters for motor controller '" << joints_[i].joint_info.name << "'!");
                            return return_type::ERROR;
                        }

                        joints_[i].joint_info.command_interface_name = hardware_interface::HW_IF_VELOCITY;
                    }
                    else
                    {
                        RCLCPP_ERROR_STREAM(rclcpp::get_logger(HW_NAME), "Unknown command interface " << interface << "!");
                        return return_type::ERROR;
                    }

                    RCLCPP_INFO_STREAM(rclcpp::get_logger(HW_NAME), "Initialize command interface of motor controller '" << joints_[i].joint_info.name << "' to '" << joints_[i].joint_info.command_interface_name << "'");
                }
            }
        }
        return return_type::OK;
    }

    CallbackReturn MotionmindHardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_DEBUG(rclcpp::get_logger(HW_NAME), "start");
        for (size_t i = 0; i < joints_.size(); i++)
        {
            if (use_dummy_ && std::isnan(joints_[i].state.position))
            {
                joints_[i].state.position = 0.0;
                joints_[i].state.velocity = 0.0;
            }
        }
        read(rclcpp::Time{}, rclcpp::Duration(0, 0));
        reset_command();
        write(rclcpp::Time{}, rclcpp::Duration(0, 0));

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn MotionmindHardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_DEBUG(rclcpp::get_logger(HW_NAME), "stop");

        // TODO: move stop code here once https://github.com/ros-controls/ros2_control/issues/472 is solved
        if (stop() != return_type::OK)
        {
            return CallbackReturn::ERROR;
        }

        return CallbackReturn::SUCCESS;
    }

    return_type MotionmindHardware::read(const rclcpp::Time & /* time */, const rclcpp::Duration & /* period */)
    {
        if (use_dummy_)
        {
            return return_type::OK;
        }

        for (size_t i = 0; i < joints_.size(); i++)
        {
            MotionmindStatus status;

            if (joints_[i].joint_info.command_interface_name == hardware_interface::HW_IF_POSITION)
            {
                int encoder_position;
                readStatusPosition(joints_[i].joint_info.address, encoder_position, status);
                joints_[i].state.position = (static_cast<double>(joints_[i].joint_info.home - encoder_position) / 214) * M_PI_2;
            }
            else if (joints_[i].joint_info.command_interface_name == hardware_interface::HW_IF_VELOCITY)
            {
                int velocity;
                readStatusVelocity(joints_[i].joint_info.address, velocity, status);

                // Unit is RPM > Check if this is valid
                if (joints_[i].joint_info.encoder_type == "4x")
                {
                    joints_[i].state.position = ((50 * velocity) / (joints_[i].joint_info.cpr * joints_[i].joint_info.gear_ratio)) * PI2;
                }
                else
                {
                    joints_[i].state.position = ((200 * velocity) / (joints_[i].joint_info.cpr * joints_[i].joint_info.gear_ratio)) * PI2;
                }
            }

            joints_[i].state.neglimit = status.NEGLIMIT;
            joints_[i].state.poslimit = status.POSLIMIT;
            joints_[i].state.brake = status.BRAKE;
            joints_[i].state.index = status.INDEX;
            joints_[i].state.badrc = status.BADRC;
            joints_[i].state.vnlimit = status.VNLIMIT;
            joints_[i].state.vplimit = status.VPLIMIT;
            joints_[i].state.currentlimit = status.CURRENTLIMIT;
            joints_[i].state.pwmlimit = status.PWMLIMIT;
            joints_[i].state.inposition = status.INPOSITION;
            joints_[i].state.tempfault = status.TEMPFAULT;
        }

        return return_type::OK;
    }

    return_type MotionmindHardware::write(const rclcpp::Time & /* time */, const rclcpp::Duration & /* period */)
    {
        if (use_dummy_)
        {
            return return_type::OK;
        }

        for (size_t i = 0; i < joints_.size(); i++)
        {
            if (joints_[i].joint_info.command_interface_name == hardware_interface::HW_IF_POSITION)
            {
                int req_encoder_position = joints_[i].joint_info.home - (joints_[i].command.position / M_PI_2) * 214;

                if (req_encoder_position > joints_[i].joint_info.min && req_encoder_position < joints_[i].joint_info.max)
                {
                    writePosition(joints_[i].joint_info.address, req_encoder_position);
                }
                else
                {
                    RCLCPP_WARN_STREAM(rclcpp::get_logger(HW_NAME), "Requested position (" << req_encoder_position << ") of Motionmind controller " << joints_[i].joint_info.name << " is not valid!");
                }
            }
            else
            {
                double req_velocity;
                if (joints_[i].joint_info.encoder_type == "4x")
                {
                    req_velocity = ((joints_[i].command.velocity / PI2) * joints_[i].joint_info.cpr * joints_[i].joint_info.gear_ratio) / 50;
                }
                else
                {
                    req_velocity = ((joints_[i].command.velocity / PI2) * joints_[i].joint_info.cpr * joints_[i].joint_info.gear_ratio) / 200;
                }
                writeVelocity(joints_[i].joint_info.address, req_velocity);
            }
        }

        return return_type::OK;
    }

    return_type MotionmindHardware::stop()
    {
        if (use_dummy_)
        {
            return return_type::OK;
        }

        // Set velocity to zero on shutdown
        for (size_t i = 0; i < joints_.size(); i++)
        {
            if (joints_[i].joint_info.command_interface_name == hardware_interface::HW_IF_VELOCITY)
            {
                writeVelocity(joints_[i].joint_info.address, 0);
            }
        }

        return return_type::OK;
    }

    return_type MotionmindHardware::reset_command()
    {
        for (size_t i = 0; i < joints_.size(); i++)
        {
            joints_[i].command.position = joints_[i].state.position;
            joints_[i].command.velocity = 0.0;
        }

        return return_type::OK;
    }

    return_type MotionmindHardware::checkMode(uint8_t address, uint8_t expected_mode)
    {
        std::vector<uint8_t> read_mode_cmd, mode_result;
        std::bitset<32> registry_index;
        registry_index[REGISTER_MODE.index] = 1;

        read_mode_cmd = createCommand(CMD_READ.cmd, address, static_cast<int>(registry_index.to_ulong()), CMD_READ.size);

        serial_connection_.flush();
        serial_connection_.write(read_mode_cmd);
        serial_connection_.read(mode_result, REGISTER_MODE.response_size + 2); // address + value + checksum

        if (mode_result.size() != REGISTER_MODE.response_size + 2)
        {
            RCLCPP_WARN_STREAM(rclcpp::get_logger(HW_NAME), "Response of Motionmind controller " << static_cast<int>(address) << " has not the expected size!");
        }

        if (!checkChecksum(mode_result))
        {
            RCLCPP_WARN_STREAM(rclcpp::get_logger(HW_NAME), "Checksum of response on Motionmind controller " << static_cast<int>(address) << " is not valid!");
        }

        if (mode_result[1] == expected_mode)
        {
            return return_type::OK;
        }

        RCLCPP_FATAL_STREAM(rclcpp::get_logger(HW_NAME), "Motor controller with address " << address << " is not in the expected mode (analog PID control for position command interface, serial PID control for velocity command interface)!");
        return return_type::ERROR;
    }

    return_type MotionmindHardware::readStatusPosition(uint8_t address, int &position, MotionmindStatus &status)
    {
        std::vector<uint8_t> read_status_cmd, status_result;
        std::bitset<32> registry_index;
        registry_index[REGISTER_POSITION.index] = 1;
        registry_index[REGISTER_STATUS.index] = 1;

        read_status_cmd = createCommand(CMD_READ.cmd, address, static_cast<int>(registry_index.to_ulong()), CMD_READ.size);

        serial_connection_.flush();
        serial_connection_.write(read_status_cmd);
        serial_connection_.read(status_result, REGISTER_POSITION.response_size + REGISTER_STATUS.response_size + 2);

        if (status_result.size() != REGISTER_POSITION.response_size + REGISTER_STATUS.response_size + 2)
        {
            RCLCPP_WARN_STREAM(rclcpp::get_logger(HW_NAME), "Response of Motionmind controller " << static_cast<int>(address) << " has not the expected size!");
        }

        if (!checkChecksum(status_result))
        {
            RCLCPP_WARN_STREAM(rclcpp::get_logger(HW_NAME), "Checksum of response on Motionmind controller " << static_cast<int>(address) << " is not valid!");
        }

        position =
            (static_cast<int>(status_result[4]) << 24) |
            (static_cast<int>(status_result[3]) << 16) |
            (static_cast<int>(status_result[2]) << 8) |
            static_cast<int>(status_result[1]);

        std::bitset<16> status_bits(static_cast<int>(status_result[6]) << 8 | static_cast<int>(status_result[5]));
        status.from_bits(status_bits);

        return return_type::OK;
    }

    return_type MotionmindHardware::readStatusVelocity(uint8_t address, int &velocity, MotionmindStatus &status)
    {
        std::vector<uint8_t> read_status_cmd, status_result;
        std::bitset<32> registry_index;
        registry_index[REGISTER_VELOCITY.index] = 1;
        registry_index[REGISTER_STATUS.index] = 1;

        read_status_cmd = createCommand(CMD_READ.cmd, address, static_cast<int>(registry_index.to_ulong()), CMD_READ.size);

        serial_connection_.flush();
        serial_connection_.write(read_status_cmd);
        serial_connection_.read(status_result, REGISTER_VELOCITY.response_size + REGISTER_STATUS.response_size + 2);

        if (status_result.size() != REGISTER_VELOCITY.response_size + REGISTER_STATUS.response_size + 2)
        {
            RCLCPP_WARN_STREAM(rclcpp::get_logger(HW_NAME), "Response of Motionmind controller " << static_cast<int>(address) << " has not the expected size!");
        }

        if (!checkChecksum(status_result))
        {
            RCLCPP_WARN_STREAM(rclcpp::get_logger(HW_NAME), "Checksum of response on Motionmind controller " << static_cast<int>(address) << " is not valid!");
        }

        velocity =
            (static_cast<int>(status_result[2]) << 8) |
            static_cast<int>(status_result[1]);

        std::bitset<16> status_bits(static_cast<int>(status_result[6]) << 8 | static_cast<int>(status_result[5]));
        status.from_bits(status_bits);

        return return_type::OK;
    }

    return_type MotionmindHardware::writePosition(uint8_t address, int position)
    {
        std::vector<uint8_t> write_position_cmd, position_result;

        write_position_cmd = createCommand(CMD_MOVETO_ABSOLUTE.cmd, address, position, CMD_MOVETO_ABSOLUTE.size);

        serial_connection_.flush();
        serial_connection_.write(write_position_cmd);
        serial_connection_.read(position_result, REGISTER_POSITION.response_size + 2);

        if (position_result.size() != REGISTER_POSITION.response_size + 2)
        {
            RCLCPP_WARN_STREAM(rclcpp::get_logger(HW_NAME), "Response of Motionmind controller " << static_cast<int>(address) << " has not the expected size!");
            return return_type::ERROR;
        }

        if (!checkChecksum(position_result))
        {
            RCLCPP_WARN_STREAM(rclcpp::get_logger(HW_NAME), "Checksum of response on Motionmind controller " << static_cast<int>(address) << " is not valid!");
            return return_type::ERROR;
        }

        return return_type::OK;
    }

    return_type MotionmindHardware::writeVelocity(uint8_t address, int velocity)
    {
        std::vector<uint8_t> write_position_cmd, velocity_result;

        write_position_cmd = createCommand(CMD_MOVEAT_VELOCITY.cmd, address, velocity, CMD_MOVEAT_VELOCITY.size);

        serial_connection_.flush();
        serial_connection_.write(write_position_cmd);
        serial_connection_.read(velocity_result, REGISTER_VELOCITY.response_size + 2);

        if (velocity_result.size() != REGISTER_VELOCITY.response_size + 2)
        {
            RCLCPP_WARN_STREAM(rclcpp::get_logger(HW_NAME), "Response of Motionmind controller " << static_cast<int>(address) << " has not the expected size!");
            return return_type::ERROR;
        }

        if (!checkChecksum(velocity_result))
        {
            RCLCPP_WARN_STREAM(rclcpp::get_logger(HW_NAME), "Checksum of response on Motionmind controller " << static_cast<int>(address) << " is not valid!");
            return return_type::ERROR;
        }

        return return_type::OK;
    }

    return_type MotionmindHardware::loadPositionParameters(int joint_idx)
    {
        if (getParamInt(info_.joints[joint_idx].parameters, "home", &joints_[joint_idx].joint_info.home) != return_type::OK)
        {
            return return_type::ERROR;
        }

        int offset_limit;
        if (getParamInt(info_.joints[joint_idx].parameters, "offset_limit", &offset_limit) != return_type::OK)
        {
            return return_type::ERROR;
        }
        joints_[joint_idx].joint_info.min = joints_[joint_idx].joint_info.home - offset_limit;
        joints_[joint_idx].joint_info.max = joints_[joint_idx].joint_info.home + offset_limit;

        return return_type::OK;
    }

    return_type MotionmindHardware::loadVelocityParameters(int joint_idx)
    {
        if (getParamInt(info_.joints[joint_idx].parameters, "cpr", &joints_[joint_idx].joint_info.cpr) != return_type::OK)
        {
            return return_type::ERROR;
        }

        if (getParamInt(info_.joints[joint_idx].parameters, "gear_ratio", &joints_[joint_idx].joint_info.gear_ratio) != return_type::OK)
        {
            return return_type::ERROR;
        }

        if (getParamString(info_.joints[joint_idx].parameters, "encoder_type", &joints_[joint_idx].joint_info.encoder_type) != return_type::OK)
        {
            return return_type::ERROR;
        }

        if (joints_[joint_idx].joint_info.encoder_type != "1x" && joints_[joint_idx].joint_info.encoder_type != "4x")
        {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger(HW_NAME), "Cannot initialize hardware controller for joint '" << joints_[joint_idx].joint_info.name << "', 1x and 4x encoder types are allowed!");
            return return_type::ERROR;
        }

        return return_type::OK;
    }

} // namespace motionmind_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    motionmind_hardware::MotionmindHardware, hardware_interface::SystemInterface)
