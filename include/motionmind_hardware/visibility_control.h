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

#ifndef MOTIONMIND_HARDWARE__VISIBLITY_CONTROL_H_
#define MOTIONMIND_HARDWARE__VISIBLITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define MOTIONMIND_HARDWARE_EXPORT __attribute__((dllexport))
#define MOTIONMIND_HARDWARE_IMPORT __attribute__((dllimport))
#else
#define MOTIONMIND_HARDWARE_EXPORT __declspec(dllexport)
#define MOTIONMIND_HARDWARE_IMPORT __declspec(dllimport)
#endif
#ifdef MOTIONMIND_HARDWARE_BUILDING_DLL
#define MOTIONMIND_HARDWARE_PUBLIC MOTIONMIND_HARDWARE_EXPORT
#else
#define MOTIONMIND_HARDWARE_PUBLIC MOTIONMIND_HARDWARE_IMPORT
#endif
#define MOTIONMIND_HARDWARE_PUBLIC_TYPE MOTIONMIND_HARDWARE_PUBLIC
#define MOTIONMIND_HARDWARE_LOCAL
#else
#define MOTIONMIND_HARDWARE_EXPORT __attribute__((visibility("default")))
#define MOTIONMIND_HARDWARE_IMPORT
#if __GNUC__ >= 4
#define MOTIONMIND_HARDWARE_PUBLIC __attribute__((visibility("default")))
#define MOTIONMIND_HARDWARE_LOCAL __attribute__((visibility("hidden")))
#else
#define MOTIONMIND_HARDWARE_PUBLIC
#define MOTIONMIND_HARDWARE_LOCAL
#endif
#define MOTIONMIND_HARDWARE_PUBLIC_TYPE
#endif

#endif // MOTIONMIND_HARDWARE__VISIBLITY_CONTROL_H_