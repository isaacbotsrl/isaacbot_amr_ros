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

#ifndef ISAACBOT_AMR_HARDWARE_INTERFACE_DOMAIN_VELOCITY_COMMAND_GUARD_HPP_
#define ISAACBOT_AMR_HARDWARE_INTERFACE_DOMAIN_VELOCITY_COMMAND_GUARD_HPP_

#include <vector>

namespace isaacbot_amr_hardware_interface
{

// Default tolerance (rad/s at the wheel) below which a velocity command counts as "zero" for the
// purposes of the E-Stop reset invariant (see EmergencyStop::resetEStop()). At the A1's
// wheel_radius of 0.1651 m this is ~1.65 mm/s - far below anything that can actually move the
// isaacbot, and far above the floating-point residue a controller's kinematics leave behind.
//
// This exists because the predicate used to compare against std::numeric_limits<double>::
// epsilon() (2.2e-16), which is an *exact* zero test, not a deadband. Any controller output that
// merely rounded near zero made the E-Stop unresettable.
constexpr double kDefaultVelocityCommandZeroTolerance = 0.01;

// Default tolerance (rad/s at the wheel) below which a *measured* wheel velocity counts as
// "not moving", for the second half of the same invariant.
//
// The command check alone is a weak guarantee: it only sees what a controller asked for, and
// anything between the command and the wheel (a controller with internal state, a board still
// ramping down, a wheel being pushed) can leave the isaacbot moving at a zero command. Measured
// velocity checks the real hazard - clearing the E-Stop while the isaacbot is still rolling -
// directly rather than by proxy. 0.05 rad/s is ~8 mm/s at the A1's 0.1651 m wheel radius, far
// below walking pace and above the noise of the DCC1020's velocity feedback at rest (TBD: check
// against the measured noise floor on the isaacbot).
constexpr double kDefaultVelocityStateZeroTolerance = 0.05;

// True when every command is finite and within `tolerance` of zero. A non-finite command (NaN
// from an uninitialized/faulted controller, inf) returns false: `std::abs(NaN) > tolerance` is
// false, so a naive comparison would let NaN silently pass the guard and clear the E-Stop while
// the commanded velocity is undefined. Fail safe instead.
//
// An empty command vector is vacuously zero - a system with no joints commands no motion.
bool areVelocitiesWithinTolerance(const std::vector<double> & commands, const double tolerance);

}  // namespace isaacbot_amr_hardware_interface

#endif  // ISAACBOT_AMR_HARDWARE_INTERFACE_DOMAIN_VELOCITY_COMMAND_GUARD_HPP_
