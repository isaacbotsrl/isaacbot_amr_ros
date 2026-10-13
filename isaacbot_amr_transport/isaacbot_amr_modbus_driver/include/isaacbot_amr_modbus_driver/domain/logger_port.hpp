// Copyright 2026 Mechatronics Academy
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

#ifndef ISAACBOT_AMR_MODBUS_DRIVER_DOMAIN_LOGGER_PORT_HPP_
#define ISAACBOT_AMR_MODBUS_DRIVER_DOMAIN_LOGGER_PORT_HPP_

#include <string>

namespace isaacbot::transport::modbus
{

// A logging seam so the client stays ROS-free.
//
// ModbusDiscreteIoClient previously held an rclcpp::Logger member purely to feed four
// RCLCPP_ERROR calls, which dragged rclcpp into what is otherwise pure logic and forced
// every unit test to stand up a ROS context. Same reasoning as the optional log_warning
// on isaacbot_amr_hardware_interface's operationWithAttempts().
//
// RclcppLogger (infrastructure) routes these to /rosout; StdErrLogger is the default for
// callers that have no ROS node.
class LoggerPort
{

public:

    virtual ~LoggerPort() = default;

    virtual void warn(const std::string & message) = 0;

    virtual void error(const std::string & message) = 0;
};

class StdErrLogger : public LoggerPort
{

public:

    void warn(const std::string & message) override;

    void error(const std::string & message) override;
};

}  // namespace isaacbot::transport::modbus

#endif  // ISAACBOT_AMR_MODBUS_DRIVER_DOMAIN_LOGGER_PORT_HPP_
