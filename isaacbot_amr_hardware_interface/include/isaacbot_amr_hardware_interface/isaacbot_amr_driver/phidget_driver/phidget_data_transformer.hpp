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

#ifndef ISAACBOT_AMR_HARDWARE_INTERFACE_ISAACBOT_AMR_DRIVER_PHIDGET_DRIVER_PHIDGET_DATA_TRANSFORMER_HPP_
#define ISAACBOT_AMR_HARDWARE_INTERFACE_ISAACBOT_AMR_DRIVER_PHIDGET_DRIVER_PHIDGET_DATA_TRANSFORMER_HPP_

#include <algorithm>

#include "isaacbot_amr_hardware_interface/utils.hpp"

namespace isaacbot_amr_hardware_interface
{

// The [-1, 1] normalized command range is a Phidget motor-controller SDK
// convention, not a generic vendor-neutral concept, so this class stays
// Phidget-named unlike the rest of the driver-state types
// (see driver_data_snapshot.hpp).
class PhidgetVelocityCommandDataTransformer
{

public:

    explicit PhidgetVelocityCommandDataTransformer(const DrivetrainSettings & drivetrain_settings);

    float convert(const float cmd) const;

private:

    static constexpr float kMaxPhidgetCmdValue = 1.0f;

    inline float clampVelCmd(const float cmd) const
    {
        return std::clamp(cmd, -kMaxPhidgetCmdValue, kMaxPhidgetCmdValue);
    }

    float radians_per_second_to_phidget_cmd_;
};

}  // namespace isaacbot_amr_hardware_interface

#endif  // ISAACBOT_AMR_HARDWARE_INTERFACE_ISAACBOT_AMR_DRIVER_PHIDGET_DRIVER_PHIDGET_DATA_TRANSFORMER_HPP_
