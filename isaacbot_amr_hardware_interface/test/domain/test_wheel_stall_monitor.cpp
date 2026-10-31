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

#include <chrono>
#include <limits>
#include <vector>

#include "isaacbot_amr_hardware_interface/domain/wheel_stall_monitor.hpp"

namespace isaacbot_amr_hardware_interface
{
namespace test
{

using namespace std::chrono_literals;

class WheelStallMonitorTest : public ::testing::Test
{
protected:
    // Defaults: |target| >= 0.2 rad/s, |measured| < 0.3 rad/s, |current| >= 8 A, hold 1 s, dropout 100 ms.
    WheelStallMonitor monitor;
    std::chrono::steady_clock::time_point t0 {};

    const std::vector<float> targets {3.0f, 0.0f, 0.0f, 0.0f};
    const std::vector<double> blocked {0.0, 0.0, 0.0, 0.0};
    const std::vector<double> high_current {13.4, 0.0, 0.0, 0.0};

    // Feeds the same observation every 20 ms from `from` up to and including `to`. Returns
    // whether any update latched.
    bool hold(
        const std::vector<float> & cmd, const std::vector<double> & measured,
        const std::vector<double> & currents, const std::chrono::milliseconds from,
        const std::chrono::milliseconds to, const bool powered = true)
    {
        bool latched = false;
        for (auto t = from; t <= to; t += 20ms) {
            latched = monitor.update(cmd, measured, currents, powered, t0 + t) || latched;
        }
        return latched;
    }
};

TEST_F(WheelStallMonitorTest, BlockedWheelLatchesAfterTheHold)
{
    EXPECT_FALSE(hold(targets, blocked, high_current, 0ms, 980ms));
    EXPECT_FALSE(monitor.isLatched());

    EXPECT_TRUE(hold(targets, blocked, high_current, 1000ms, 1000ms));
    EXPECT_TRUE(monitor.isLatched());
    EXPECT_EQ(monitor.latchedWheel(), 0);
    EXPECT_DOUBLE_EQ(monitor.latchedTarget(), 3.0);
    EXPECT_DOUBLE_EQ(monitor.latchedMeasured(), 0.0);
    EXPECT_DOUBLE_EQ(monitor.latchedCurrent(), 13.4);
}

TEST_F(WheelStallMonitorTest, ReportsTheLatchOnlyOnce)
{
    ASSERT_TRUE(hold(targets, blocked, high_current, 0ms, 1000ms));
    EXPECT_FALSE(hold(targets, blocked, high_current, 1020ms, 3000ms));
    EXPECT_TRUE(monitor.isLatched());
}

TEST_F(WheelStallMonitorTest, StaysLatchedOnceTheWheelIsStopped)
{
    ASSERT_TRUE(hold(targets, blocked, high_current, 0ms, 1000ms));

    const std::vector<float> zero(4, 0.0f);
    const std::vector<double> no_current(4, 0.0);
    EXPECT_FALSE(hold(zero, blocked, no_current, 1020ms, 5000ms));
    EXPECT_TRUE(monitor.isLatched());
}

TEST_F(WheelStallMonitorTest, HardStartFromRestDoesNotLatch)
{
    // Ground runs 2026-10-31: wheels leave 0.3 rad/s within about 0.1 s at peak current.
    EXPECT_FALSE(hold(targets, blocked, high_current, 0ms, 100ms));
    EXPECT_FALSE(hold(targets, {2.0, 0.0, 0.0, 0.0}, high_current, 120ms, 3000ms));
    EXPECT_FALSE(monitor.isLatched());
}

TEST_F(WheelStallMonitorTest, MovingWheelAtHighCurrentIsNotAStall)
{
    // A scrubbing in-place turn: well above the speed threshold, at high current.
    EXPECT_FALSE(hold(targets, {1.5, 0.0, 0.0, 0.0}, high_current, 0ms, 5000ms));
}

TEST_F(WheelStallMonitorTest, SlowWheelAtLowCurrentIsNotAStall)
{
    // Crawling at 0.05 m/s: about 1 A on the ground.
    const std::vector<float> crawl {0.3f, 0.0f, 0.0f, 0.0f};
    EXPECT_FALSE(hold(crawl, {0.25, 0.0, 0.0, 0.0}, {1.0, 0.0, 0.0, 0.0}, 0ms, 5000ms));
}

TEST_F(WheelStallMonitorTest, ZeroTargetIsNotAStall)
{
    const std::vector<float> zero(4, 0.0f);
    EXPECT_FALSE(hold(zero, blocked, high_current, 0ms, 5000ms));
}

TEST_F(WheelStallMonitorTest, NegativeTargetAndRegenerativeCurrentCount)
{
    const std::vector<float> reverse {-3.0f, 0.0f, 0.0f, 0.0f};
    EXPECT_TRUE(hold(reverse, blocked, {-13.4, 0.0, 0.0, 0.0}, 0ms, 1000ms));
}

TEST_F(WheelStallMonitorTest, BriefDropoutDoesNotRestartTheHold)
{
    EXPECT_FALSE(hold(targets, blocked, high_current, 0ms, 500ms));
    // Current ripple: under the threshold for 60 ms, inside the 100 ms dropout.
    EXPECT_FALSE(hold(targets, blocked, {5.0, 0.0, 0.0, 0.0}, 520ms, 580ms));
    EXPECT_TRUE(hold(targets, blocked, high_current, 600ms, 1000ms));
}

TEST_F(WheelStallMonitorTest, LongerRecoveryRestartsTheHold)
{
    EXPECT_FALSE(hold(targets, blocked, high_current, 0ms, 800ms));
    // The wheel broke free for longer than the dropout.
    EXPECT_FALSE(hold(targets, {2.0, 0.0, 0.0, 0.0}, high_current, 820ms, 1000ms));
    EXPECT_FALSE(hold(targets, blocked, high_current, 1020ms, 1800ms));
    EXPECT_FALSE(monitor.isLatched());
    EXPECT_TRUE(hold(targets, blocked, high_current, 1820ms, 2020ms));
}

TEST_F(WheelStallMonitorTest, NotEvaluatedWhileUnpowered)
{
    EXPECT_FALSE(hold(targets, blocked, high_current, 0ms, 5000ms, false));
    EXPECT_FALSE(monitor.isLatched());
}

TEST_F(WheelStallMonitorTest, LosingPowerRestartsTheHold)
{
    EXPECT_FALSE(hold(targets, blocked, high_current, 0ms, 900ms));
    EXPECT_FALSE(hold(targets, blocked, high_current, 920ms, 920ms, false));
    EXPECT_FALSE(hold(targets, blocked, high_current, 940ms, 1800ms));
    EXPECT_FALSE(monitor.isLatched());
}

TEST_F(WheelStallMonitorTest, NonFiniteSamplesAreNoEvidence)
{
    const double nan = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(hold(targets, blocked, {nan, nan, nan, nan}, 0ms, 5000ms));
    EXPECT_FALSE(hold(targets, {nan, nan, nan, nan}, high_current, 5020ms, 10000ms));
    EXPECT_FALSE(monitor.isLatched());
}

TEST_F(WheelStallMonitorTest, EveryWheelIsWatched)
{
    const std::vector<float> rear_right {0.0f, 0.0f, 0.0f, 3.0f};
    EXPECT_TRUE(hold(rear_right, blocked, {0.0, 0.0, 0.0, 13.4}, 0ms, 1000ms));
    EXPECT_EQ(monitor.latchedWheel(), 3);
}

TEST_F(WheelStallMonitorTest, ResetClearsTheLatchAndTheReadings)
{
    ASSERT_TRUE(hold(targets, blocked, high_current, 0ms, 1000ms));

    monitor.reset();

    EXPECT_FALSE(monitor.isLatched());
    EXPECT_EQ(monitor.latchedWheel(), WheelStallMonitor::kNoWheel);
    EXPECT_DOUBLE_EQ(monitor.latchedCurrent(), 0.0);

    // Still blocked after the reset: it needs a full hold again, then latches again.
    EXPECT_FALSE(hold(targets, blocked, high_current, 1020ms, 2000ms));
    EXPECT_TRUE(hold(targets, blocked, high_current, 2020ms, 2020ms));
}

}  // namespace test
}  // namespace isaacbot_amr_hardware_interface
