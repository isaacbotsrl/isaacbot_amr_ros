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

#ifndef ISAACBOT_AMR_HARDWARE_INTERFACE_ISAACBOT_AMR_DRIVER_ARDENNES_DRIVER_HPP_
#define ISAACBOT_AMR_HARDWARE_INTERFACE_ISAACBOT_AMR_DRIVER_ARDENNES_DRIVER_HPP_

#include <chrono>
#include <memory>
#include <vector>

#include "isaacbot_amr_hardware_interface/isaacbot_amr_driver/phidget_driver/phidget_isaacbot_amr_driver.hpp"
#include "isaacbot_amr_hardware_interface/isaacbot_amr_driver/phidget_driver/phidget_motor_driver.hpp"

namespace isaacbot_amr_hardware_interface
{

class ArdennesDriver : public PhidgetIsaacbotDriver
{

public:

    ArdennesDriver(
        const DrivetrainSettings & drivetrain_settings,
        const std::chrono::milliseconds activate_wait_time = std::chrono::milliseconds(1000));

    ~ArdennesDriver() = default;

    void sendSpeedCmd(const std::vector<float> & speeds);

    void attemptErrorFlagReset();

protected:

    void defineDrivers() override;

private:

    // Reused by attemptErrorFlagReset() so it never allocates a temporary std::vector on the RT
    // path (IsaacbotSystem::read() -> updateFlagErrors() -> attemptErrorFlagReset(), called every
    // cycle while a fault flag is active).
    std::vector<float> zero_speed_cmd_ = {0.0f, 0.0f, 0.0f, 0.0f};
};

}  // namespace isaacbot_amr_hardware_interface

#endif  // ISAACBOT_AMR_HARDWARE_INTERFACE_ISAACBOT_AMR_DRIVER_ARDENNES_DRIVER_HPP_
