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

#ifndef ISAACBOT_AMR_HARDWARE_INTERFACE_UTILS_HPP_
#define ISAACBOT_AMR_HARDWARE_INTERFACE_UTILS_HPP_

#include <chrono>
#include <cstdint>
#include <functional>
#include <optional>
#include <stdexcept>
#include <string>

namespace isaacbot_amr_hardware_interface
{

// Default for DrivetrainSettings::motor_current_limit - the previously hard-coded 10 A limit.
// Deliberately not "board default": the DCC1020's own default is far above what the drive motors
// are rated for.
constexpr float kDefaultMotorCurrentLimit = 10.0f;

struct DrivetrainSettings
{
    float motor_torque_constant;
    float gear_ratio;
    float gearbox_efficiency;
    float encoder_resolution;
    float max_rpm_motor_speed;
    unsigned driver_comm_timeout_ms;
    // Scales a driver's raw current feedback reading into amps before it's multiplied by
    // motor_torque_constant/gear_ratio/gearbox_efficiency to get torque (see
    // MotorStateReading::getTorque() in domain/driver_data_snapshot.cpp). This is a property of
    // the driver backend's raw feedback units, not of the drivetrain itself - e.g. Phidget
    // driver reports current in milliamps (PhidgetMotorDriver::currentChangeHandler), so the URDF supplies
    // 0.001 here. A different driver backend reporting raw current in a different unit supplies
    // its own value instead of silently inheriting Phidget's convention.
    float raw_current_to_amps_scale;
    // Timeout (ms) for the motor driver's own hardware watchdog (e.g. Phidget's
    // PhidgetMotorVelocityController_enableFailsafe()) - see MotorDriverInterface::armFailsafe() in domain/
    // driver.hpp. Optional in the URDF; absent means kDefaultMotorFailsafeTimeoutMs, so an
    // existing URDF keeps working unchanged.
    unsigned motor_failsafe_timeout_ms;
    // Motor controller current limit in A (e.g. PhidgetMotorVelocityController_setCurrentLimit()).
    // Optional in the URDF.
    float motor_current_limit{kDefaultMotorCurrentLimit};

    // Settings of the motor controller's on-board velocity loop (DCC1020
    // MotorVelocityController). The controller is rescaled to motor-shaft RPM, so every velocity
    // below is motor RPM and every acceleration RPM/s. All optional in the URDF: an absent value
    // (std::nullopt) leaves the board's own default in place, and the driver logs what it ended
    // up with - gains are tuned on the robot, never invented here.
    std::optional<float> velocity_kp;
    std::optional<float> velocity_ki;
    std::optional<float> velocity_kd;
    // Target-velocity band around 0 in which the controller disengages instead of holding the
    // motor still (only applies to a zero target).
    std::optional<float> velocity_deadband_rpm;
    // Trapezoidal-profile acceleration limit, RPM/s.
    std::optional<float> motor_acceleration_rpm_s;
    // Current the controller may exceed motor_current_limit by for ~1 s, A.
    std::optional<float> motor_surge_current_limit;
    // Below this speed under load the controller declares a dangerous stall and disengages; 0
    // turns stall protection off. RPM.
    std::optional<float> motor_stall_velocity_rpm;
    // Current limit applied while the controller is in its failsafe state, A.
    std::optional<float> motor_failsafe_current_limit;
    // Motor inductance in the SDK's unit (bounded by getMin/MaxInductance()). Absent: the board
    // measures it itself when the channel opens (the audible beeps).
    std::optional<float> motor_inductance;
    // Whether the controller brakes (true) or coasts (false) when its failsafe trips. Defaults
    // to braking, which is what the DCC1000 setup did (braking strength 1.0); the board's own
    // default is to coast.
    bool motor_failsafe_braking_enabled{true};

    // Whether the motor driver's own hardware watchdog (motor_failsafe_timeout_ms above) is armed.
    // Optional in the URDF; absent means enabled, so every existing URDF keeps its protection.
    // Only an instrumented build (logging/tracing that can stall the RT loop past the timeout)
    // should ever set this to false: with it off a stalled write() loop leaves each wheel at its
    // last velocity target instead of braking it.
    bool motor_failsafe_enabled{true};
};

constexpr unsigned kDefaultMotorFailsafeTimeoutMs = 500;

// `log_warning`, if provided, receives each attempt-failure message instead of the default
// std::cerr fallback - lets infrastructure callers (e.g. IsaacbotSystem, which has an rclcpp::Logger)
// route these through RCLCPP_WARN_STREAM instead. Left optional (default nullptr -> std::cerr)
// rather than a hard rclcpp dependency here, since this header is transitively included from
// domain/driver_data_snapshot.hpp and must stay ROS-free (see the comment in utils.cpp and
// scripts/check_domain_purity.sh).
bool operationWithAttempts(
    const std::function<void()> operation,
    const unsigned max_attempts,
    const std::function<void()> on_error = []() {},
    const std::chrono::milliseconds delay_between_attempts = std::chrono::milliseconds(0),
    const std::function<void(const std::string &)> & log_warning = nullptr);

bool checkIfJointNameContainValidSequence(
    const std::string & name,
    const std::string & sequence);

}  // namespace isaacbot_amr_hardware_interface

#endif  // ISAACBOT_AMR_HARDWARE_INTERFACE_UTILS_HPP_
