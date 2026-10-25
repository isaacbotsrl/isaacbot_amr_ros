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

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <limits>
#include <vector>

#include "isaacbot_amr_hardware_interface/domain/wheel_runaway_monitor.hpp"

namespace isaacbot_amr_hardware_interface
{
namespace test
{

using namespace std::chrono_literals;

class WheelRunawayMonitorTest : public ::testing::Test
{
protected:
    // Defaults: margin max(1.0, 0.5 * |target|) rad/s, hold 300 ms, convergence 0.25 rad/s.
    WheelRunawayMonitor monitor;
    std::chrono::steady_clock::time_point t0 {};

    // Feeds the same observation every 20 ms from `from` up to and including `to`. Returns
    // whether any update latched.
    bool hold(
        const std::vector<float> & targets, const std::vector<double> & measured,
        const std::chrono::milliseconds from, const std::chrono::milliseconds to, const bool powered = true)
    {
        bool latched = false;
        for (auto t = from; t <= to; t += 20ms) {
            latched = monitor.update(targets, measured, powered, t0 + t) || latched;
        }
        return latched;
    }
};

TEST_F(WheelRunawayMonitorTest, ExcessIsOverspeedOrAnythingTheWrongWay)
{
    EXPECT_DOUBLE_EQ(WheelRunawayMonitor::excess(1.0, 1.5), 0.5);
    EXPECT_DOUBLE_EQ(WheelRunawayMonitor::excess(-1.0, -1.5), 0.5);
    EXPECT_DOUBLE_EQ(WheelRunawayMonitor::excess(1.0, 0.4), 0.0);      // slower is not excess
    EXPECT_DOUBLE_EQ(WheelRunawayMonitor::excess(0.6, -13.0), 13.0);   // wrong way
    EXPECT_DOUBLE_EQ(WheelRunawayMonitor::excess(0.0, -2.0), 2.0);     // moving at a zero target
}

TEST_F(WheelRunawayMonitorTest, LatchesTheFrontLeftRunawaySeenOnTheArdennes)
{
    // 2026-10-25: FL encoder against its motor; target 0.606 rad/s, FL read -13 rad/s.
    const std::vector<float> targets {0.606f, 0.606f, 0.606f, 0.606f};
    const std::vector<double> measured {-13.0, 0.6, 0.6, 0.6};

    EXPECT_FALSE(hold(targets, measured, 0ms, 280ms));
    EXPECT_FALSE(monitor.isLatched());
    EXPECT_TRUE(monitor.update(targets, measured, true, t0 + 300ms));
    EXPECT_TRUE(monitor.isLatched());
    EXPECT_EQ(monitor.latchedWheel(), 0);
    EXPECT_NEAR(monitor.latchedTarget(), 0.606, 1e-6);
    EXPECT_DOUBLE_EQ(monitor.latchedMeasured(), -13.0);
}

TEST_F(WheelRunawayMonitorTest, KeepsRunningAtAZeroTargetLatches)
{
    // The same runaway after the command went back to zero.
    EXPECT_TRUE(hold({0.0f}, {-13.0}, 0ms, 400ms));
    EXPECT_EQ(monitor.latchedWheel(), 0);
}

TEST_F(WheelRunawayMonitorTest, StepOvershootDoesNotLatch)
{
    // Worst tuned overshoot: 17 % at 0.303 rad/s, 8 % at 3 rad/s - both inside the margin.
    EXPECT_FALSE(hold({0.303f}, {0.355}, 0ms, 2000ms));
    EXPECT_FALSE(hold({3.0f}, {3.24}, 0ms, 2000ms));
    EXPECT_FALSE(monitor.isLatched());
}

TEST_F(WheelRunawayMonitorTest, SlowerThanTargetNeverLatches)
{
    EXPECT_FALSE(hold({5.0f}, {0.0}, 0ms, 2000ms));   // stalled wheel
    EXPECT_FALSE(monitor.isLatched());
}

TEST_F(WheelRunawayMonitorTest, BriefExcessShorterThanHoldDoesNotLatch)
{
    EXPECT_FALSE(hold({0.0f}, {2.0}, 0ms, 200ms));
    EXPECT_FALSE(hold({0.0f}, {0.0}, 220ms, 300ms));
    EXPECT_FALSE(hold({0.0f}, {2.0}, 320ms, 520ms));   // the timer restarted at 320 ms
    EXPECT_FALSE(monitor.isLatched());
}

TEST_F(WheelRunawayMonitorTest, DecelerationAfterAStopDoesNotLatch)
{
    // Target drops to zero from 3 rad/s; the wheel ramps down at ~5 rad/s^2 (0.1 rad/s per
    // 20 ms cycle) - excess above the margin for 400 ms, but converging the whole time.
    bool latched = false;
    double measured = 3.0;
    for (auto t = 0ms; t <= 800ms; t += 20ms) {
        latched = monitor.update({0.0f}, {measured}, true, t0 + t) || latched;
        measured = std::max(0.0, measured - 0.1);
    }
    EXPECT_FALSE(latched);
    EXPECT_FALSE(monitor.isLatched());
}

TEST_F(WheelRunawayMonitorTest, NotEvaluatedWhileMotorsUnpowered)
{
    // Pushed by hand while E-Stopped: the contactor is open, nothing can run away.
    EXPECT_FALSE(hold({0.0f}, {3.0}, 0ms, 1000ms, false));
    EXPECT_FALSE(monitor.isLatched());
}

TEST_F(WheelRunawayMonitorTest, PowerLossClearsARunningEpisode)
{
    EXPECT_FALSE(hold({0.0f}, {3.0}, 0ms, 200ms));
    EXPECT_FALSE(monitor.update({0.0f}, {3.0}, false, t0 + 220ms));
    // Power back: a fresh episode, so 200 ms more is still not enough.
    EXPECT_FALSE(hold({0.0f}, {3.0}, 240ms, 440ms));
    EXPECT_FALSE(monitor.isLatched());
}

TEST_F(WheelRunawayMonitorTest, StaysLatchedUntilResetAndReportsOnlyOnce)
{
    EXPECT_TRUE(hold({0.0f}, {3.0}, 0ms, 300ms));
    EXPECT_FALSE(hold({0.0f}, {0.0}, 320ms, 1000ms));   // stopped - but still latched
    EXPECT_TRUE(monitor.isLatched());

    monitor.reset();
    EXPECT_FALSE(monitor.isLatched());
    EXPECT_EQ(monitor.latchedWheel(), WheelRunawayMonitor::kNoWheel);
    EXPECT_TRUE(hold({0.0f}, {3.0}, 1000ms, 1300ms));   // and can latch again
}

TEST_F(WheelRunawayMonitorTest, ReportsTheFirstWheelToLatch)
{
    const std::vector<float> targets {0.0f, 0.0f, 0.0f, 0.0f};
    EXPECT_FALSE(hold(targets, {0.0, 0.0, 3.0, 0.0}, 0ms, 100ms));
    EXPECT_TRUE(hold(targets, {0.0, 3.0, 3.0, 0.0}, 120ms, 400ms));
    EXPECT_EQ(monitor.latchedWheel(), 2);
}

TEST_F(WheelRunawayMonitorTest, NonFiniteSamplesAreNoEvidence)
{
    const double nan = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(hold({0.0f}, {nan}, 0ms, 1000ms));
    EXPECT_FALSE(hold({0.0f}, {std::numeric_limits<double>::infinity()}, 1020ms, 2000ms));
    EXPECT_FALSE(monitor.isLatched());

    // And a NaN in the middle breaks an episode rather than carrying it.
    EXPECT_FALSE(hold({0.0f}, {3.0}, 2020ms, 2200ms));
    EXPECT_FALSE(hold({0.0f}, {nan}, 2220ms, 2220ms));
    EXPECT_FALSE(hold({0.0f}, {3.0}, 2240ms, 2440ms));
    EXPECT_FALSE(monitor.isLatched());
}

TEST_F(WheelRunawayMonitorTest, IgnoresWheelsBeyondTheShorterInput)
{
    EXPECT_FALSE(hold({0.0f}, {0.0, 5.0}, 0ms, 1000ms));
    EXPECT_FALSE(monitor.isLatched());
}

}  // namespace test
}  // namespace isaacbot_amr_hardware_interface
