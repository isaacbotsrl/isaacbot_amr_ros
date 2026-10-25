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

#include "isaacbot_amr_hardware_interface/domain/wheel_runaway_monitor.hpp"

#include <algorithm>
#include <cmath>

namespace isaacbot_amr_hardware_interface
{

WheelRunawayMonitor::WheelRunawayMonitor(const WheelRunawaySettings & settings)
: settings_(settings)
{

}

double WheelRunawayMonitor::excess(const double target, const double measured)
{
    const bool same_direction = (target > 0.0 && measured > 0.0) || (target < 0.0 && measured < 0.0);

    if (same_direction) {
        return std::max(0.0, std::fabs(measured) - std::fabs(target));
    }

    // Zero target or the wrong way: everything measured is excess.
    return std::fabs(measured);
}

bool WheelRunawayMonitor::update(
    const std::vector<float> & targets, const std::vector<double> & measured,
    const bool motors_powered, const std::chrono::steady_clock::time_point now)
{
    if (!motors_powered) {
        clearEpisodes();
        return false;
    }

    const std::size_t count = std::min({targets.size(), measured.size(), kMaxWheels});
    bool latched_now = false;

    for (std::size_t i = 0; i < count; ++i) {
        const double target = static_cast<double>(targets[i]);
        Episode & episode = episodes_[i];

        // No evidence either way: invalid data is the communication-timeout faults' business, not
        // a reason to cut motor power.
        if (!std::isfinite(target) || !std::isfinite(measured[i])) {
            episode.active = false;
            continue;
        }

        const double over = excess(target, measured[i]);
        const double margin = std::max(settings_.min_margin_rad_s, settings_.target_ratio * std::fabs(target));

        if (over <= margin) {
            episode.active = false;
            continue;
        }

        if (!episode.active || over < episode.reference_excess - settings_.convergence_rad_s) {
            // A new episode, or one that is converging: (re)start the hold from here.
            episode.active = true;
            episode.since = now;
            episode.reference_excess = over;
            continue;
        }

        if ((now - episode.since) >= settings_.hold && !latched_.load()) {
            latched_wheel_ = static_cast<int>(i);
            latched_target_ = target;
            latched_measured_ = measured[i];
            latched_ = true;
            latched_now = true;
        }
    }

    return latched_now;
}

bool WheelRunawayMonitor::isLatched() const
{
    return latched_.load();
}

int WheelRunawayMonitor::latchedWheel() const
{
    return latched_wheel_.load();
}

double WheelRunawayMonitor::latchedTarget() const
{
    return latched_target_.load();
}

double WheelRunawayMonitor::latchedMeasured() const
{
    return latched_measured_.load();
}

void WheelRunawayMonitor::reset()
{
    clearEpisodes();
    latched_wheel_ = kNoWheel;
    latched_target_ = 0.0;
    latched_measured_ = 0.0;
    latched_ = false;
}

void WheelRunawayMonitor::clearEpisodes()
{
    for (auto & episode : episodes_) {
        episode.active = false;
    }
}

}  // namespace isaacbot_amr_hardware_interface
