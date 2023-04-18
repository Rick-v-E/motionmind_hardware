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

#ifndef MOTIONMIND_DEFINITIONS_H_
#define MOTIONMIND_DEFINITIONS_H_

#include <bitset>

struct MotionmindCMD
{
    uint8_t cmd;
    unsigned int size;
};

struct MotionmindRegistry
{
    unsigned int index;
    unsigned int response_size;
};

struct MotionmindStatus
{
    bool NEGLIMIT;
    bool POSLIMIT;
    bool BRAKE;
    bool INDEX;
    bool BADRC;
    bool VNLIMIT;
    bool VPLIMIT;
    bool CURRENTLIMIT;
    bool PWMLIMIT;
    bool INPOSITION;
    bool TEMPFAULT;

    MotionmindStatus() {}

    void from_bits(std::bitset<16> status_bits)
    {
        NEGLIMIT = (status_bits[0] == 1);
        POSLIMIT = (status_bits[1] == 1);
        BRAKE = (status_bits[2] == 1);
        INDEX = (status_bits[3] == 1);
        BADRC = (status_bits[4] == 1);
        VNLIMIT = (status_bits[5] == 1);
        VPLIMIT = (status_bits[6] == 1);
        CURRENTLIMIT = (status_bits[7] == 1);
        PWMLIMIT = (status_bits[8] == 1);
        INPOSITION = (status_bits[9] == 1);
        TEMPFAULT = (status_bits[10] == 1);
    }
};

const MotionmindCMD CMD_CHANGE_SPEED = {20, 2};
const MotionmindCMD CMD_MOVETO_ABSOLUTE = {21, 4};
const MotionmindCMD CMD_MOVETO_RELATIVE = {22, 4};
const MotionmindCMD CMD_MOVEAT_VELOCITY = {23, 2};
const MotionmindCMD CMD_WRITE = {24, 4};
const MotionmindCMD CMD_WRITE_STORE = {25, 4};
const MotionmindCMD CMD_READ = {26, 4};
const MotionmindCMD CMD_RESTORE = {27, 0};
const MotionmindCMD CMD_RESET = {28, 0};
const MotionmindCMD CMD_READ_REGISTER = {29, 1};

const MotionmindRegistry REGISTER_POSITION = {0, 4};
const MotionmindRegistry REGISTER_VELOCITYLIMIT = {1, 2};
const MotionmindRegistry REGISTER_VELOCITYFF = {2, 1};
const MotionmindRegistry REGISTER_FUNCTION = {3, 2};
const MotionmindRegistry REGISTER_PTERM = {4, 2};
const MotionmindRegistry REGISTER_ITERM = {5, 2};
const MotionmindRegistry REGISTER_DTERM = {6, 2};
const MotionmindRegistry REGISTER_ADDRESS = {7, 1};
const MotionmindRegistry REGISTER_PIDSCALAR = {8, 1};
const MotionmindRegistry REGISTER_TIMER = {9, 2};
const MotionmindRegistry REGISTER_RCMAX = {10, 2};
const MotionmindRegistry REGISTER_RCMIN = {11, 2};
const MotionmindRegistry REGISTER_RCBAND = {12, 2};
const MotionmindRegistry REGISTER_RCCOUNT = {13, 2};
const MotionmindRegistry REGISTER_VELOCITY = {14, 2};
const MotionmindRegistry REGISTER_TIME = {15, 4};
const MotionmindRegistry REGISTER_STATUS = {16, 2};
const MotionmindRegistry REGISTER_REVISION = {17, 1};
const MotionmindRegistry REGISTER_MODE = {18, 1};
const MotionmindRegistry REGISTER_ANALOGCON = {19, 2};
const MotionmindRegistry REGISTER_ANALOGFBCK = {20, 2};
const MotionmindRegistry REGISTER_PWMOUT = {21, 2};
const MotionmindRegistry REGISTER_INDEXPOS = {22, 4};
const MotionmindRegistry REGISTER_VNLIMIT = {23, 4};
const MotionmindRegistry REGISTER_VPLIMIT = {24, 4};
const MotionmindRegistry REGISTER_PWMLIMIT = {25, 2};
const MotionmindRegistry REGISTER_DEADBAND = {26, 2};
const MotionmindRegistry REGISTER_DESIREDPOSITION = {27, 4};
const MotionmindRegistry REGISTER_AMPSLIMIT = {28, 2};
const MotionmindRegistry REGISTER_AMPS = {29, 2};
const MotionmindRegistry REGISTER_FUNCTION2 = {30, 2};
const MotionmindRegistry REGISTER_THERMISTOR = {31, 2};

#endif