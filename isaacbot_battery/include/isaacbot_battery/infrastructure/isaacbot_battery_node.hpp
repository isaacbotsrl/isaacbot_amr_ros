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

#ifndef ISAACBOT_BATTERY_INFRASTRUCTURE_ISAACBOT_BATTERY_NODE_HPP_
#define ISAACBOT_BATTERY_INFRASTRUCTURE_ISAACBOT_BATTERY_NODE_HPP_

#include <chrono>
#include <memory>
#include <string>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "rclcpp/rclcpp.hpp"
#include "udp_msgs/msg/udp_packet.hpp"

#include "isaacbot_battery/application/monitor_battery_use_case.hpp"
#include "isaacbot_battery/domain/battery_report.hpp"

namespace isaacbot_battery
{

/**
 * @brief Composition root: decodes BMS UDP packets and drives MonitorBatteryUseCase.
 * @details Plain (non-lifecycle) node on purpose — the BMS link is owned by the isaacbot_udp_driver
 *          lifecycle node; this node only transforms its output.
 */
class IsaacbotBatteryNode : public rclcpp::Node
{
public:
    IsaacbotBatteryNode(
        const std::string & node_name, const std::string & ns = "/",
        const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

    // For rclcpp_components: constructs and runs init(). The container's __node / __ns remaps
    // set the name and namespace the launch file gives it.
    explicit IsaacbotBatteryNode(const rclcpp::NodeOptions & options);

    void init();

private:
    void batteryUdpDataCallback(const udp_msgs::msg::UdpPacket::SharedPtr msg);

    void batteryUdpDataSubscriberTimeoutCallback();

    domain::BatteryIdentity identity_;
    std::chrono::milliseconds watchdog_timeout_;

    std::unique_ptr<application::MonitorBatteryUseCase> monitor_battery_;

    rclcpp::Subscription<udp_msgs::msg::UdpPacket>::SharedPtr battery_subscriber_;
    rclcpp::TimerBase::SharedPtr battery_read_timeout_;

    // Declared last so it is destroyed first: it holds raw pointers to the publisher's
    // diagnostic callbacks, and the publisher is owned by monitor_battery_.
    std::shared_ptr<diagnostic_updater::Updater> diagnostic_updater_;
};

}  // namespace isaacbot_battery

#endif  // ISAACBOT_BATTERY_INFRASTRUCTURE_ISAACBOT_BATTERY_NODE_HPP_
