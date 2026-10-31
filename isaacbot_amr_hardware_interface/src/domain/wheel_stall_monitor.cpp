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

#include "isaacbot_amr_hardware_interface/domain/wheel_stall_monitor.hpp"

#include <algorithm>
#include <cmath>

namespace isaacbot_amr_hardware_interface
{

WheelStallMonitor::WheelStallMonitor(const WheelStallSettings & settings)
: settings_(settings)
{

}

bool WheelStallMonitor::update(
    const std::vector<float> & targets, const std::vector<double> & measured,
    const std::vector<double> & currents, const bool motors_powered,
    const std::chrono::steady_clock::time_point now)
{
    if (!motors_powered) {
        clearEpisodes();
        return false;
    }

    const std::size_t count = std::min({targets.size(), measured.size(), currents.size(), kMaxWheels});
    bool latched_now = false;

    for (std::size_t i = 0; i < count; ++i) {
        const double target = static_cast<double>(targets[i]);
        Episode & episode = episodes_[i];

        // No evidence either way: invalid data is the communication-timeout faults' business.
        if (!std::isfinite(target) || !std::isfinite(measured[i]) || !std::isfinite(currents[i])) {
            episode.active = false;
            continue;
        }

        const bool stalled = std::fabs(target) >= settings_.min_target_rad_s &&
                             std::fabs(measured[i]) < settings_.max_speed_rad_s &&
                             std::fabs(currents[i]) >= settings_.min_current_a;

        if (!stalled) {
            if (episode.active && (now - episode.last_seen) > settings_.dropout) {
                episode.active = false;
            }
            continue;
        }

        if (!episode.active) {
            episode.active = true;
            episode.since = now;
        }

        episode.last_seen = now;

        if ((now - episode.since) >= settings_.hold && !latched_.load()) {
            latched_wheel_ = static_cast<int>(i);
            latched_target_ = target;
            latched_measured_ = measured[i];
            latched_current_ = currents[i];
            latched_ = true;
            latched_now = true;
        }
    }

    return latched_now;
}

bool WheelStallMonitor::isLatched() const
{
    return latched_.load();
}

int WheelStallMonitor::latchedWheel() const
{
    return latched_wheel_.load();
}

double WheelStallMonitor::latchedTarget() const
{
    return latched_target_.load();
}

double WheelStallMonitor::latchedMeasured() const
{
    return latched_measured_.load();
}

double WheelStallMonitor::latchedCurrent() const
{
    return latched_current_.load();
}

void WheelStallMonitor::reset()
{
    clearEpisodes();
    latched_wheel_ = kNoWheel;
    latched_target_ = 0.0;
    latched_measured_ = 0.0;
    latched_current_ = 0.0;
    latched_ = false;
}

void WheelStallMonitor::clearEpisodes()
{
    for (auto & episode : episodes_) {
        episode.active = false;
    }
}

}  // namespace isaacbot_amr_hardware_interface
