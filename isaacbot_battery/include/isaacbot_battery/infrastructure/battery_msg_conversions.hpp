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

#ifndef ISAACBOT_BATTERY_INFRASTRUCTURE_BATTERY_MSG_CONVERSIONS_HPP_
#define ISAACBOT_BATTERY_INFRASTRUCTURE_BATTERY_MSG_CONVERSIONS_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include "isaacbot_msgs/msg/charging_status.hpp"
#include "sensor_msgs/msg/battery_state.hpp"

#include "isaacbot_battery/domain/battery_report.hpp"

namespace isaacbot_battery::infrastructure
{

using BatteryStateMsg = sensor_msgs::msg::BatteryState;
using ChargingStatusMsg = isaacbot_msgs::msg::ChargingStatus;

std::uint8_t toPowerSupplyHealth(domain::BatteryHealth health);
std::uint8_t toPowerSupplyStatus(domain::ChargeState state);
std::uint8_t toChargerType(domain::ChargerType type);

/** Header stamp / frame are left empty, as they always have been on this topic. */
BatteryStateMsg toBatteryStateMsg(const domain::BatteryReading & reading);
ChargingStatusMsg toChargingStatusMsg(const domain::ChargingInfo & info);

/** Joins alarm names into the single string shown in logs and diagnostics. */
std::string joinErrors(const std::vector<std::string> & errors);

}  // namespace isaacbot_battery::infrastructure

#endif  // ISAACBOT_BATTERY_INFRASTRUCTURE_BATTERY_MSG_CONVERSIONS_HPP_
