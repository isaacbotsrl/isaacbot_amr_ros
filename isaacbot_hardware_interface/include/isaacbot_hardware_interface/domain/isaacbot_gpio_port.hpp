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

#ifndef ISAACBOT_HARDWARE_INTERFACE_DOMAIN_ISAACBOT_GPIO_PORT_HPP_
#define ISAACBOT_HARDWARE_INTERFACE_DOMAIN_ISAACBOT_GPIO_PORT_HPP_

#include <unordered_map>

#include "isaacbot_hardware_interface/domain/isaacbot_gpio_types.hpp"
#include "isaacbot_hardware_interface/domain/safety_link_health.hpp"

namespace isaacbot_hardware_interface
{

// Port: the GPIO/safety-controller primitives IsaacbotSystem needs to start the controller, drive
// the aux outputs, poll its IO state and report link health, without depending on the concrete
// Modbus-backed IsaacbotSafetyController directly. It deliberately has no E-Stop coil writes: those
// belong to EmergencyStopIoPort, behind EmergencyStop's rules. Implemented by
// IsaacbotSafetyControllerGpioAdapter (see
// isaacbot_safety_controller/isaacbot_safety_controller_gpio_adapter.hpp), mirroring how
// EmergencyStopIoPort/IsaacbotSafetyControllerEStopIo isolate EmergencyStop from the same concrete
// type.
class IsaacbotGpioPort
{

public:

    virtual ~IsaacbotGpioPort() = default;

    virtual void start() = 0;

    // Drives general-purpose output GPIO_AUX_OUT_<index> (index < kAuxOutputCount). NOT RT-safe:
    // blocks for a Modbus round-trip and throws on failure, an out-of-range index, or before
    // start(). Call from a service thread only, never from read()/write().
    virtual void setAuxOutput(const unsigned index, const bool state) = 0;

    // Non-blocking; returns a reference to a cache owned by the implementation. Safe to call
    // from the RT thread (see IsaacbotSafetyController::queryControlInterfaceIOStates()).
    virtual const std::unordered_map<IsaacbotControllerGpio, bool> & queryControlInterfaceIOStates() = 0;

    // Health of the link and of the threads servicing it, for the diagnostics task. Non-blocking.
    virtual SafetyLinkHealth linkHealth() const = 0;
};

}  // namespace isaacbot_hardware_interface

#endif  // ISAACBOT_HARDWARE_INTERFACE_DOMAIN_ISAACBOT_GPIO_PORT_HPP_
