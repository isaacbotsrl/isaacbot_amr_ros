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

#include "isaacbot_amr_hardware_interface/isaacbot_amr_safety_controller/isaacbot_amr_safety_controller_gpio_adapter.hpp"

#include <utility>

namespace isaacbot_amr_hardware_interface
{

IsaacbotSafetyControllerGpioAdapter::IsaacbotSafetyControllerGpioAdapter(
    std::shared_ptr<IsaacbotSafetyController> isaacbot_amr_controller)
: isaacbot_amr_controller_(std::move(isaacbot_amr_controller))
{

}

void IsaacbotSafetyControllerGpioAdapter::start()
{
    isaacbot_amr_controller_->start();
}

void IsaacbotSafetyControllerGpioAdapter::setAuxOutput(const unsigned index, const bool state)
{
    isaacbot_amr_controller_->setAuxOutput(index, state);
}

const std::unordered_map<IsaacbotControllerGpio, bool> &
IsaacbotSafetyControllerGpioAdapter::queryControlInterfaceIOStates()
{
    return isaacbot_amr_controller_->queryControlInterfaceIOStates();
}

SafetyLinkHealth IsaacbotSafetyControllerGpioAdapter::linkHealth() const
{
    return isaacbot_amr_controller_->getHealth();
}

}  // namespace isaacbot_amr_hardware_interface
