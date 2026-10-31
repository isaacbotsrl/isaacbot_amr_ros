// Copyright 2026 IsaacBot
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

#ifndef ISAACBOT_AMR_HARDWARE_INTERFACE_DOMAIN_WHEEL_STALL_MONITOR_HPP_
#define ISAACBOT_AMR_HARDWARE_INTERFACE_DOMAIN_WHEEL_STALL_MONITOR_HPP_

#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <vector>

namespace isaacbot_amr_hardware_interface
{

struct WheelStallSettings
{
    // A commanded wheel slower than this is not turning. 0.3 rad/s is the wheel speed at 0.05 m/s,
    // the slowest motion measured on the ground (2026-10-31), where it drew about 1 A.
    double max_speed_rad_s = 0.3;

    // Below this the motor is not working hard enough to be at risk. Ground runs (2026-10-31): 1 A
    // cruising at low speed, 5-8 A scrubbing through a turn (at wheel speeds above max_speed_rad_s).
    double min_current_a = 8.0;

    // Targets smaller than this are not a request to turn. Just below the 0.30 rad/s of 0.05 m/s.
    double min_target_rad_s = 0.2;

    // How long the signature must last before it latches. Longer than the 0.1 s it takes a wheel
    // to start moving from rest at full acceleration, so a hard start or a brief breakaway rides
    // through; short enough to stop before the current has been at its limit for long.
    std::chrono::milliseconds hold {1000};

    // A gap shorter than this does not restart the hold: current ripple or one stick-slip sample
    // must not reset the timer of a wheel that is still blocked.
    std::chrono::milliseconds dropout {100};
};

// Detects a wheel that is commanded to turn, is drawing high current, and is not turning: blocked,
// jammed, or its gearbox seized. The board's StallVelocity protection covers the same fault from
// its own duty-cycle model, which is opaque and can't be tested without stalling a motor; this
// works from what was measured, and stays in place if that protection is switched off.
//
// Unlike a runaway, a stall obeys a zero command, so the caller only stops commanding the wheels.
// A zero target is never a stall: inside velocity_deadband_rpm the board lets go of the motor.
// Evaluated only while the motors are powered. Non-finite samples are skipped: stale or invalid
// data is caught by the communication-timeout faults.
//
// Pure domain logic - `now` is injected rather than read from a clock, and update() is
// allocation-free, so it can run on the RT read() cycle.
class WheelStallMonitor
{

public:

    static constexpr std::size_t kMaxWheels = 8;
    static constexpr int kNoWheel = -1;

    explicit WheelStallMonitor(const WheelStallSettings & settings = WheelStallSettings {});

    // Feeds one observation per wheel: what was last sent (`targets`, joint rad/s), the velocity
    // read back (`measured`, joint rad/s) and the motor current (`currents`, A, either sign), same
    // index order. Wheels beyond kMaxWheels are ignored. Returns true only on the cycle the fault
    // latches.
    bool update(
        const std::vector<float> & targets, const std::vector<double> & measured,
        const std::vector<double> & currents, const bool motors_powered,
        const std::chrono::steady_clock::time_point now);

    // True from the latching update() until reset(). Latched because the zero commands that follow
    // remove the signature: the fault must not look cleared just because the wheel was stopped.
    bool isLatched() const;

    // The first wheel that latched, and its readings at that moment. kNoWheel and zeros while not
    // latched. For logs and diagnostics.
    int latchedWheel() const;
    double latchedTarget() const;
    double latchedMeasured() const;
    double latchedCurrent() const;

    // Clears the latch and every per-wheel episode. Wired to the same operator action that clears
    // the relay latch.
    void reset();

private:

    struct Episode
    {
        bool active = false;
        std::chrono::steady_clock::time_point since {};
        std::chrono::steady_clock::time_point last_seen {};
    };

    void clearEpisodes();

    const WheelStallSettings settings_;

    // Touched only from the single read() cycle. The published values are atomic because
    // diagnostics reads them from another thread.
    std::array<Episode, kMaxWheels> episodes_ {};

    std::atomic_bool latched_ {false};
    std::atomic_int latched_wheel_ {kNoWheel};
    std::atomic<double> latched_target_ {0.0};
    std::atomic<double> latched_measured_ {0.0};
    std::atomic<double> latched_current_ {0.0};
};

}  // namespace isaacbot_amr_hardware_interface

#endif  // ISAACBOT_AMR_HARDWARE_INTERFACE_DOMAIN_WHEEL_STALL_MONITOR_HPP_
