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

#ifndef ISAACBOT_AMR_HARDWARE_INTERFACE_ISAACBOT_AMR_SAFETY_CONTROLLER_ISAACBOT_AMR_SAFETY_CONTROLLER_GPIO_ADAPTER_HPP_
#define ISAACBOT_AMR_HARDWARE_INTERFACE_ISAACBOT_AMR_SAFETY_CONTROLLER_ISAACBOT_AMR_SAFETY_CONTROLLER_GPIO_ADAPTER_HPP_

#include <memory>
#include <unordered_map>

#include "isaacbot_amr_hardware_interface/domain/isaacbot_amr_gpio_port.hpp"
#include "isaacbot_amr_hardware_interface/isaacbot_amr_safety_controller/isaacbot_amr_safety_controller.hpp"

namespace isaacbot_amr_hardware_interface
{

// Infrastructure adapter: implements the domain's IsaacbotGpioPort on top of the concrete
// Modbus-backed IsaacbotSafetyController, so IsaacbotSystem can depend on the port instead of on
// IsaacbotSafetyController directly (mirrors IsaacbotSafetyControllerEStopIo for EmergencyStopIoPort).
class IsaacbotSafetyControllerGpioAdapter : public IsaacbotGpioPort
{

public:

    explicit IsaacbotSafetyControllerGpioAdapter(std::shared_ptr<IsaacbotSafetyController> isaacbot_amr_controller);

    void start() override;

    void setAuxOutput(const unsigned index, const bool state) override;

    const std::unordered_map<IsaacbotControllerGpio, bool> & queryControlInterfaceIOStates() override;

    SafetyLinkHealth linkHealth() const override;

private:

    std::shared_ptr<IsaacbotSafetyController> isaacbot_amr_controller_;
};

}  // namespace isaacbot_amr_hardware_interface

#endif  // ISAACBOT_AMR_HARDWARE_INTERFACE_ISAACBOT_AMR_SAFETY_CONTROLLER_ISAACBOT_AMR_SAFETY_CONTROLLER_GPIO_ADAPTER_HPP_
