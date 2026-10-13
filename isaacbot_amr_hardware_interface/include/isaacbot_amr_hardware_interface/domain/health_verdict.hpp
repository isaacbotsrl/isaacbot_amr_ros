// Copyright 2025 Mechatronics Academy
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

#ifndef ISAACBOT_HARDWARE_INTERFACE_DOMAIN_HEALTH_VERDICT_HPP_
#define ISAACBOT_HARDWARE_INTERFACE_DOMAIN_HEALTH_VERDICT_HPP_

#include <string>

namespace isaacbot_amr_hardware_interface
{

// Severity of a diagnostics verdict, independent of ROS: the infrastructure layer maps it onto
// diagnostic_msgs/DiagnosticStatus levels.
enum class HealthLevel
{
    kOk,
    kWarn,
    kError,
};

struct HealthVerdict
{
    HealthLevel level{HealthLevel::kOk};
    std::string message;
};

}  // namespace isaacbot_amr_hardware_interface

#endif  // ISAACBOT_HARDWARE_INTERFACE_DOMAIN_HEALTH_VERDICT_HPP_
