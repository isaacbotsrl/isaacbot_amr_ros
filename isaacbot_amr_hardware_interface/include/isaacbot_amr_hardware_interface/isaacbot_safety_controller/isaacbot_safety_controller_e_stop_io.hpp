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

#ifndef ISAACBOT_HARDWARE_INTERFACE_ISAACBOT_SAFETY_CONTROLLER_ISAACBOT_SAFETY_CONTROLLER_E_STOP_IO_HPP_
#define ISAACBOT_HARDWARE_INTERFACE_ISAACBOT_SAFETY_CONTROLLER_ISAACBOT_SAFETY_CONTROLLER_E_STOP_IO_HPP_

#include <memory>

#include "isaacbot_amr_hardware_interface/domain/emergency_stop.hpp"
#include "isaacbot_amr_hardware_interface/isaacbot_safety_controller/isaacbot_safety_controller.hpp"

namespace isaacbot_amr_hardware_interface
{

// Infrastructure adapter: implements the domain's EmergencyStopIoPort on top of the concrete
// Modbus-backed IsaacbotSafetyController, so EmergencyStop itself never depends on IsaacbotSafetyController.
class IsaacbotSafetyControllerEStopIo : public EmergencyStopIoPort
{

public:

    explicit IsaacbotSafetyControllerEStopIo(std::shared_ptr<IsaacbotSafetyController> isaacbot_amr_controller);

    bool isUserButtonActive() override;

    bool isLatchActive() override;

    bool isContactorEngaged() override;

    void triggerUserButton(const bool state) override;

    void triggerMotorDriverFault(const bool state) override;

    void resetLatch() override;

private:

    std::shared_ptr<IsaacbotSafetyController> isaacbot_amr_controller_;
};

}  // namespace isaacbot_amr_hardware_interface

#endif  // ISAACBOT_HARDWARE_INTERFACE_ISAACBOT_SAFETY_CONTROLLER_ISAACBOT_SAFETY_CONTROLLER_E_STOP_IO_HPP_
