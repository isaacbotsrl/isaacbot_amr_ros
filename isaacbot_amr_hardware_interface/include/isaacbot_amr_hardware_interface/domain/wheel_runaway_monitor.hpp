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

#ifndef ISAACBOT_AMR_HARDWARE_INTERFACE_DOMAIN_WHEEL_RUNAWAY_MONITOR_HPP_
#define ISAACBOT_AMR_HARDWARE_INTERFACE_DOMAIN_WHEEL_RUNAWAY_MONITOR_HPP_

#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <vector>

namespace isaacbot_amr_hardware_interface
{

struct WheelRunawaySettings
{
    // A wheel counts as running away once it exceeds its target by more than
    // max(min_margin_rad_s, target_ratio * |target|): fixed near standstill, proportional at speed.
    // Tuned DCC1020 steps overshoot by under 20 %, so 50 % of target leaves room for that.
    double min_margin_rad_s = 1.0;
    double target_ratio = 0.5;

    // How long the excess must last, without converging, before it latches. Rides through step
    // overshoot (settled within ~0.4 s on the Ardennes) and single bad samples.
    std::chrono::milliseconds hold {300};

    // An excess that has dropped by this much since the episode started is converging - an
    // ordinary deceleration after a stop - and restarts the hold timer instead of latching.
    double convergence_rad_s = 0.25;
};

// Detects a wheel that is driving itself: faster than commanded, or the wrong way, and not coming
// back. The case it exists for: a DCC1020 whose encoder counts against its motor turns its own
// velocity loop into positive feedback and saturates at full speed. Commanding zero does not stop
// that - the loop runs on the board - so the only remedy is cutting motor power, which is what the
// caller does on a latch.
//
// Only overspeed counts. A wheel slower than its target (load, stall, ramp-up) is not a runaway.
// Evaluated only while the motors are powered: with the contactor open nothing can run away, and
// a wheel turned by hand or rolling while E-Stopped must not latch a fault. Non-finite samples
// are skipped: stale or invalid data is caught by the communication-timeout faults.
//
// Pure domain logic - `now` is injected rather than read from a clock, and update() is
// allocation-free, so it can run on the RT read() cycle.
class WheelRunawayMonitor
{

public:

    static constexpr std::size_t kMaxWheels = 8;
    static constexpr int kNoWheel = -1;

    explicit WheelRunawayMonitor(const WheelRunawaySettings & settings = WheelRunawaySettings {});

    // Feeds one observation per wheel: `targets` is what was last sent to each wheel, `measured`
    // the velocity read back, same index order and units (joint rad/s). Wheels beyond kMaxWheels
    // are ignored. Returns true only on the cycle the fault latches.
    bool update(
        const std::vector<float> & targets, const std::vector<double> & measured,
        const bool motors_powered, const std::chrono::steady_clock::time_point now);

    // True from the latching update() until reset(). Latched deliberately, like the welded-contactor
    // fault: a wheel that ran away once must not look healthy again because power was cut.
    bool isLatched() const;

    // The first wheel that latched, and its target and measured velocity at that moment. kNoWheel
    // and zeros while not latched. For logs and diagnostics.
    int latchedWheel() const;
    double latchedTarget() const;
    double latchedMeasured() const;

    // Clears the latch and every per-wheel episode. Wired to the same operator action that clears
    // the relay latch.
    void reset();

    // How far `measured` runs ahead of `target` (rad/s, >= 0): the overspeed in the commanded
    // direction, or all of `measured` against a zero target or the wrong way. Exposed for tests.
    static double excess(const double target, const double measured);

private:

    struct Episode
    {
        bool active = false;
        std::chrono::steady_clock::time_point since {};
        double reference_excess = 0.0;
    };

    void clearEpisodes();

    const WheelRunawaySettings settings_;

    // Touched only from the single read() cycle. The published values are atomic because
    // diagnostics reads them from another thread.
    std::array<Episode, kMaxWheels> episodes_ {};

    std::atomic_bool latched_ {false};
    std::atomic_int latched_wheel_ {kNoWheel};
    std::atomic<double> latched_target_ {0.0};
    std::atomic<double> latched_measured_ {0.0};
};

}  // namespace isaacbot_amr_hardware_interface

#endif  // ISAACBOT_AMR_HARDWARE_INTERFACE_DOMAIN_WHEEL_RUNAWAY_MONITOR_HPP_
