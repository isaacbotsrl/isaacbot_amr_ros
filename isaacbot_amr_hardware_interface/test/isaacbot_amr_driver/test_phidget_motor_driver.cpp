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

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <utility>

#include "isaacbot_amr_hardware_interface/domain/driver_data_snapshot.hpp"
#include "isaacbot_amr_hardware_interface/isaacbot_amr_driver/phidget_driver/phidget_motor_driver.hpp"
#include "isaacbot_amr_hardware_interface/isaacbot_amr_driver/phidget_driver/phidget_utils.hpp"

namespace isaacbot_amr_hardware_interface
{

TEST(PhidgetMotorDriverFailsafeTest, TrippedReturnCodeIsRecognized)
{
    EXPECT_TRUE(PhidgetMotorDriver::isFailsafeTrippedReturnCode(EPHIDGET_FAILSAFE));
}

TEST(PhidgetMotorDriverFailsafeTest, OtherReturnCodesAreNotTreatedAsTripped)
{
    EXPECT_FALSE(PhidgetMotorDriver::isFailsafeTrippedReturnCode(EPHIDGET_OK));
    EXPECT_FALSE(PhidgetMotorDriver::isFailsafeTrippedReturnCode(EPHIDGET_TIMEOUT));
    EXPECT_FALSE(PhidgetMotorDriver::isFailsafeTrippedReturnCode(EPHIDGET_NOTATTACHED));
}

using FailsafeAction = PhidgetMotorDriver::FailsafeAction;

TEST(PhidgetMotorDriverFailsafeTest, NeverEnabledChannelIsEnabled)
{
    EXPECT_EQ(
        PhidgetMotorDriver::selectFailsafeAction(true, false, false), FailsafeAction::kEnable);
}

TEST(PhidgetMotorDriverFailsafeTest, AlreadyEnabledHealthyChannelIsOnlyFed)
{
    // Re-enabling an already-enabled failsafe on an open channel is rejected by the SDK.
    EXPECT_EQ(
        PhidgetMotorDriver::selectFailsafeAction(true, true, false), FailsafeAction::kFeed);
}

TEST(PhidgetMotorDriverFailsafeTest, TrippedChannelIsReopened)
{
    // A tripped channel rejects both resetFailsafe and enableFailsafe until it is re-opened.
    EXPECT_EQ(
        PhidgetMotorDriver::selectFailsafeAction(true, true, true),
        FailsafeAction::kReopenAndEnable);
    EXPECT_EQ(
        PhidgetMotorDriver::selectFailsafeAction(true, false, true),
        FailsafeAction::kReopenAndEnable);
}

TEST(PhidgetMotorDriverFailsafeTest, ConfiguredOffNeverArmsFeedsOrReopens)
{
    // motor_failsafe_enabled=false (instrumented build): whatever the channel state, armFailsafe()
    // must do nothing - in particular it must not re-open a "tripped" channel to re-arm it.
    for (const bool enabled : {false, true}) {
        for (const bool tripped : {false, true}) {
            EXPECT_EQ(
                PhidgetMotorDriver::selectFailsafeAction(false, enabled, tripped),
                FailsafeAction::kNone)
                << "enabled=" << enabled << " tripped=" << tripped;
        }
    }
}

TEST(PhidgetMotorDriverEncoderTest, StaleTimeoutSpansSeveralDataIntervals)
{
    using std::chrono::milliseconds;
    // A turning wheel must survive one late velocity event.
    EXPECT_EQ(PhidgetMotorDriver::encoderStaleTimeout(50), milliseconds(150));
    EXPECT_EQ(PhidgetMotorDriver::encoderStaleTimeout(200), milliseconds(600));
}

TEST(PhidgetMotorDriverEncoderTest, StaleTimeoutHasAFloorForShortIntervals)
{
    // At a board-minimum interval of a few ms, 3x would zero a turning wheel on any SDK-thread
    // scheduling hiccup.
    EXPECT_EQ(PhidgetMotorDriver::encoderStaleTimeout(1), PhidgetMotorDriver::kMinVelocityStaleTimeout);
    EXPECT_EQ(PhidgetMotorDriver::encoderStaleTimeout(10), PhidgetMotorDriver::kMinVelocityStaleTimeout);
    EXPECT_GE(PhidgetMotorDriver::kMinVelocityStaleTimeout, std::chrono::milliseconds(80));
}

TEST(PhidgetMotorDriverTargetTest, ClampsToTheMotorSpeedLimit)
{
    EXPECT_FLOAT_EQ(PhidgetMotorDriver::motorTargetRpm(1000.0f, 2800.0f, false), 1000.0f);
    EXPECT_FLOAT_EQ(PhidgetMotorDriver::motorTargetRpm(5000.0f, 2800.0f, false), 2800.0f);
    EXPECT_FLOAT_EQ(PhidgetMotorDriver::motorTargetRpm(-5000.0f, 2800.0f, false), -2800.0f);
}

TEST(PhidgetMotorDriverTargetTest, ReversedMotorGetsTheNegatedClampedTarget)
{
    EXPECT_FLOAT_EQ(PhidgetMotorDriver::motorTargetRpm(1000.0f, 2800.0f, true), -1000.0f);
    EXPECT_FLOAT_EQ(PhidgetMotorDriver::motorTargetRpm(5000.0f, 2800.0f, true), -2800.0f);
}

TEST(PhidgetMotorDriverTargetTest, NonFiniteCommandStopsTheWheel)
{
    // The SDK rejects NaN, which would leave the board on its previous (moving) target.
    for (const float cmd : {std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::infinity(),
                            -std::numeric_limits<float>::infinity()})
    {
        EXPECT_FLOAT_EQ(PhidgetMotorDriver::motorTargetRpm(cmd, 2800.0f, false), 0.0f) << cmd;
        EXPECT_FLOAT_EQ(PhidgetMotorDriver::motorTargetRpm(cmd, 2800.0f, true), 0.0f) << cmd;
    }
}

TEST(PhidgetMotorDriverErrorEventTest, FaultFlagNamesFitTheEightBitField)
{
    // DriverState::fault_flags is a uint8_t; a ninth flag would silently never be reported.
    EXPECT_LE(FaultFlag().getErrorMap().size(), 8u);
}

TEST(PhidgetMotorDriverUnitsTest, RescaleFactorMakesTheControllerWorkInMotorRpm)
{
    // 1024 lines -> 4096 counts/rev. 4096 counts/s is 1 rev/s = 60 RPM, so one count/s must
    // rescale to 60 / 4096 RPM.
    EXPECT_DOUBLE_EQ(motorRpmRescaleFactor(1024.0f), 60.0 / 4096.0);
    EXPECT_DOUBLE_EQ(4096.0 * motorRpmRescaleFactor(1024.0f), 60.0);
    EXPECT_DOUBLE_EQ(motorRpmRescaleFactor(0.0f), 0.0);
}

TEST(PhidgetMotorDriverUnitsTest, WheelRadPerSecondConvertsThroughTheGearbox)
{
    // 2*pi rad/s at the wheel is 1 wheel rev/s = gear_ratio motor rev/s.
    EXPECT_NEAR(wheelRadPerSecToMotorRpm(2.0 * M_PI, 23.3f), 23.3 * 60.0, 1e-3);
    EXPECT_NEAR(wheelRadPerSecToMotorRpm(-1.0, 1.0f), -60.0 / (2.0 * M_PI), 1e-12);
    EXPECT_DOUBLE_EQ(wheelRadPerSecToMotorRpm(0.0, 23.3f), 0.0);
}

TEST(PhidgetMotorDriverUnitsTest, CommandAndFeedbackConversionsRoundTrip)
{
    // MotorStateReading turns the reported motor RPM back into wheel rad/s with
    // (2*pi / 60) / gear_ratio; a commanded speed must read back unchanged.
    const float gear_ratio = 23.3f;
    const double wheel = 3.7;
    const double rpm = wheelRadPerSecToMotorRpm(wheel, gear_ratio);

    EXPECT_NEAR(rpm * (2.0 * M_PI) / 60.0 / gear_ratio, wheel, 1e-9);
}

TEST(PhidgetMotorDriverPositionFallbackTest, IntegratesRpmIntoLines)
{
    // 60 RPM = 1 rev/s; 1024 lines/rev for 0.5 s is 512 lines.
    EXPECT_DOUBLE_EQ(PhidgetMotorDriver::motorRpmToLines(60.0, 0.5, 1024.0f), 512.0);
    EXPECT_DOUBLE_EQ(PhidgetMotorDriver::motorRpmToLines(-60.0, 1.0, 1024.0f), -1024.0);
    EXPECT_DOUBLE_EQ(PhidgetMotorDriver::motorRpmToLines(60.0, 0.0, 1024.0f), 0.0);
    EXPECT_DOUBLE_EQ(PhidgetMotorDriver::motorRpmToLines(60.0, 1.0, 0.0f), 0.0);
}

namespace
{

constexpr std::uint8_t bit(const std::size_t b)
{
    return static_cast<std::uint8_t>(1U << b);
}

}  // namespace

TEST(PhidgetMotorDriverErrorEventTest, EStopRaisesEmergencyStopAndLatches)
{
    const auto effect = PhidgetMotorDriver::errorEventEffect(EEPHIDGET_ESTOP);

    EXPECT_EQ(effect.set_fault_bits, bit(FaultFlag::kEmergencyStopBit));
    EXPECT_TRUE(effect.latches_failsafe);
}

TEST(PhidgetMotorDriverErrorEventTest, DisengagingFaultsLatch)
{
    const auto stall = PhidgetMotorDriver::errorEventEffect(EEPHIDGET_MOTORSTALL);
    EXPECT_EQ(stall.set_fault_bits, bit(FaultFlag::kMotorStallBit));
    EXPECT_TRUE(stall.latches_failsafe);

    const auto energy_dump = PhidgetMotorDriver::errorEventEffect(EEPHIDGET_ENERGYDUMP);
    EXPECT_EQ(energy_dump.set_fault_bits, bit(FaultFlag::kOverVoltageBit));
    EXPECT_TRUE(energy_dump.latches_failsafe);

    const auto failsafe = PhidgetMotorDriver::errorEventEffect(EEPHIDGET_FAILSAFE);
    EXPECT_EQ(failsafe.set_fault_bits, 0);
    EXPECT_TRUE(failsafe.latches_failsafe);
}

TEST(PhidgetMotorDriverErrorEventTest, TransientFaultsDoNotLatch)
{
    const std::pair<Phidget_ErrorEventCode, std::size_t> cases[] = {
        {EEPHIDGET_OVERTEMP, FaultFlag::kOverTemperatureBit},
        {EEPHIDGET_OVERCURRENT, FaultFlag::kOverCurrentBit},
        {EEPHIDGET_OVERVOLTAGE, FaultFlag::kOverVoltageBit},
        {EEPHIDGET_BADPOWER, FaultFlag::kBadPowerBit},
        {EEPHIDGET_VOLTAGEERROR, FaultFlag::kBadPowerBit},
    };

    for (const auto & [code, fault_bit] : cases) {
        const auto effect = PhidgetMotorDriver::errorEventEffect(code);

        EXPECT_EQ(effect.set_fault_bits, bit(fault_bit)) << code;
        EXPECT_FALSE(effect.latches_failsafe) << code;
        EXPECT_NE(PhidgetMotorDriver::transientFaultBits() & bit(fault_bit), 0) << code;
    }
}

TEST(PhidgetMotorDriverErrorEventTest, ErrorClearedClearsOnlyTransientFaults)
{
    const auto effect = PhidgetMotorDriver::errorEventEffect(EEPHIDGET_OK);

    EXPECT_EQ(effect.set_fault_bits, 0);
    EXPECT_EQ(effect.clear_fault_bits, PhidgetMotorDriver::transientFaultBits());
    EXPECT_FALSE(effect.latches_failsafe);
    EXPECT_EQ(effect.clear_fault_bits & bit(FaultFlag::kEmergencyStopBit), 0);
    EXPECT_EQ(effect.clear_fault_bits & bit(FaultFlag::kMotorStallBit), 0);
}

TEST(PhidgetMotorDriverErrorEventTest, UnrelatedEventsHaveNoEffect)
{
    const auto effect = PhidgetMotorDriver::errorEventEffect(EEPHIDGET_PACKETLOST);

    EXPECT_EQ(effect.set_fault_bits, 0);
    EXPECT_EQ(effect.clear_fault_bits, 0);
    EXPECT_FALSE(effect.latches_failsafe);
}

}  // namespace isaacbot_amr_hardware_interface
