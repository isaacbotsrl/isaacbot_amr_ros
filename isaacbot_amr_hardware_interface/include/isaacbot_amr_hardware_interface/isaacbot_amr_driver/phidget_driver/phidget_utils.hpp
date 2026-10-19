// Copyright 2025 IsaacBot
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

#ifndef ISAACBOT_AMR_HARDWARE_INTERFACE_ISAACBOT_AMR_DRIVER_PHIDGET_DRIVER_PHIDGET_UTILS_HPP_
#define ISAACBOT_AMR_HARDWARE_INTERFACE_ISAACBOT_AMR_DRIVER_PHIDGET_DRIVER_PHIDGET_UTILS_HPP_

#include <phidget22.h>

#include <stdexcept>
#include <string>

namespace isaacbot_amr_hardware_interface
{

#define PHIDGET_UTILS_NO_COPY_NO_MOVE_NO_ASSIGN(Classname)  \
    Classname(const Classname &) = delete;                  \
    void operator=(const Classname &) = delete;             \
    Classname(Classname &&) = delete;                       \
    void operator=(Classname &&) = delete;

void openWaitForAttachment(
    PhidgetHandle handle,
    int32_t serial_number,
    int hub_port,
    bool is_hub_port_device,
    int channel);

void closeAndDelete(PhidgetHandle * handle) noexcept;

// "0x34 (Phidget not physically attached)"-style text for a PhidgetReturnCode, for logs and
// diagnostics. Falls back to the bare hex code if the SDK has no description.
std::string phidgetReturnCodeToString(const PhidgetReturnCode ret);

// Rescale factor that makes a MotorVelocityController / MotorPositionController work in
// motor-shaft RPM: the SDK's native unit is quadrature counts (4 per encoder line), and a
// rescaled velocity is <rescaled position unit> per second, so one count must be worth
// 60 / (4 * lines) "RPM-seconds". Returns 0 for a non-positive `encoder_lines`.
double motorRpmRescaleFactor(const float encoder_lines);

// Wheel-joint angular velocity (rad/s, what diff_drive_controller commands) to motor-shaft RPM.
double wheelRadPerSecToMotorRpm(const double wheel_rad_per_s, const float gear_ratio);

}  // namespace isaacbot_amr_hardware_interface

#endif  // ISAACBOT_AMR_HARDWARE_INTERFACE_ISAACBOT_AMR_DRIVER_PHIDGET_DRIVER_PHIDGET_UTILS_HPP_
