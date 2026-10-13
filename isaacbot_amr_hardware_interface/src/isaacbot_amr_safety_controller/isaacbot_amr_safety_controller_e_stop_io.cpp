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

#include "isaacbot_amr_hardware_interface/isaacbot_amr_safety_controller/isaacbot_amr_safety_controller_e_stop_io.hpp"

#include <utility>

namespace isaacbot_amr_hardware_interface
{

IsaacbotSafetyControllerEStopIo::IsaacbotSafetyControllerEStopIo(
    std::shared_ptr<IsaacbotSafetyController> isaacbot_amr_controller)
: isaacbot_amr_controller_(std::move(isaacbot_amr_controller))
{

}

bool IsaacbotSafetyControllerEStopIo::isUserButtonActive()
{
    return isaacbot_amr_controller_->isPinActive(IsaacbotControllerGpio::GPIO_SW_E_STOP_USER_BUTTON);
}

bool IsaacbotSafetyControllerEStopIo::isLatchActive()
{
    return isaacbot_amr_controller_->isPinActive(IsaacbotControllerGpio::GPIO_SW_E_STOP_LATCH_STATUS);
}

bool IsaacbotSafetyControllerEStopIo::isContactorEngaged()
{
    return isaacbot_amr_controller_->isPinActive(IsaacbotControllerGpio::GPIO_MOTOR_CONTACTOR_ENGAGED);
}

void IsaacbotSafetyControllerEStopIo::triggerUserButton(const bool state)
{
    isaacbot_amr_controller_->eStopUserBtnTrigger(state);
}

void IsaacbotSafetyControllerEStopIo::triggerMotorDriverFault(const bool state)
{
    isaacbot_amr_controller_->eStopMotorDriverFaultTrigger(state);
}

void IsaacbotSafetyControllerEStopIo::resetLatch()
{
    isaacbot_amr_controller_->eStopLatchReset();
}

}  // namespace isaacbot_amr_hardware_interface
