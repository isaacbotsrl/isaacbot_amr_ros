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

#include <gtest/gtest.h>

#include <exception>
#include <functional>
#include <memory>
#include <utility>
#include <vector>

#include "isaacbot_amr_hardware_interface/domain/emergency_stop.hpp"

namespace isaacbot_amr_hardware_interface
{

// Thrown by the fake port on request, so a test can tell an unwrapped port exception from
// EmergencyStop's own std::runtime_error wrapping.
struct FakeIoError : std::exception
{
    const char * what() const noexcept override { return "fake IO error"; }
};

enum class PortWrite { kUserButton, kMotorDriverFault, kLatchReset };

class FakeEmergencyStopIo : public EmergencyStopIoPort
{

public:

    bool isUserButtonActive() override { return user_button_active; }

    bool isLatchActive() override { return latch_active; }

    bool isContactorEngaged() override { return contactor_engaged; }

    void triggerUserButton(const bool state) override
    {
        if (throw_on_user_button) {
            throw FakeIoError();
        }

        trigger_user_button_calls.push_back(state);
        write_log.emplace_back(PortWrite::kUserButton, state);
        user_button_active = state;
    }

    void triggerMotorDriverFault(const bool state) override
    {
        if (throw_on_motor_driver_fault) {
            throw FakeIoError();
        }

        write_log.emplace_back(PortWrite::kMotorDriverFault, state);
    }

    void requestMotorDriverFault() override
    {
        request_motor_driver_fault_calls++;
    }

    void resetLatch() override
    {
        reset_latch_calls++;
        write_log.emplace_back(PortWrite::kLatchReset, true);
        latch_active = false;
    }

    bool user_button_active = true;
    bool latch_active = true;
    bool contactor_engaged = true;
    std::vector<bool> trigger_user_button_calls;
    unsigned reset_latch_calls = 0;
    unsigned request_motor_driver_fault_calls = 0;
    // Both triggers, in call order.
    std::vector<std::pair<PortWrite, bool>> write_log;
    bool throw_on_user_button = false;
    bool throw_on_motor_driver_fault = false;
};

class EmergencyStopTest : public ::testing::Test
{
protected:
    void makeEStop(std::function<bool()> zero_velocity_check)
    {
        io = std::make_shared<FakeEmergencyStopIo>();
        e_stop = std::make_unique<EmergencyStop>(io, zero_velocity_check);
    }

    std::shared_ptr<FakeEmergencyStopIo> io;
    std::unique_ptr<EmergencyStop> e_stop;
};

TEST_F(EmergencyStopTest, ReadStateReflectsPortValues)
{
    makeEStop([]() { return true; });

    io->user_button_active = false;
    io->latch_active = true;

    EXPECT_FALSE(e_stop->readEStopState());
    EXPECT_TRUE(e_stop->readEStopLatchState());
}

TEST_F(EmergencyStopTest, SetEStopTriggersUserButton)
{
    makeEStop([]() { return true; });

    e_stop->setEStop();

    ASSERT_EQ(io->trigger_user_button_calls.size(), 1u);
    EXPECT_TRUE(io->trigger_user_button_calls[0]);
}

TEST_F(EmergencyStopTest, ResetEStopRefusesWhileVelocityCommandsAreNotZero)
{
    makeEStop([]() { return false; });

    EXPECT_THROW(e_stop->resetEStop(), std::runtime_error);
    // Must not have touched the IO port at all - the invariant is checked before any I/O.
    EXPECT_TRUE(io->trigger_user_button_calls.empty());
}

TEST_F(EmergencyStopTest, ResetEStopSucceedsWhenVelocityCommandsAreZero)
{
    makeEStop([]() { return true; });

    e_stop->resetEStop();

    ASSERT_EQ(io->trigger_user_button_calls.size(), 1u);
    EXPECT_FALSE(io->trigger_user_button_calls[0]);
}

TEST_F(EmergencyStopTest, ResetEStopWithNoZeroVelocityCheckSkipsTheGuard)
{
    makeEStop(nullptr);

    e_stop->resetEStop();

    ASSERT_EQ(io->trigger_user_button_calls.size(), 1u);
    EXPECT_FALSE(io->trigger_user_button_calls[0]);
}

TEST_F(EmergencyStopTest, ResetEStopLatchResetsThePort)
{
    makeEStop([]() { return true; });

    e_stop->resetEStopLatch();

    EXPECT_EQ(io->reset_latch_calls, 1u);
}

// The relay latch is set-dominant: a motor-driver-fault input still asserted (by the runaway
// guard) would set it again the moment the reset pulse ends.
TEST_F(EmergencyStopTest, ResetEStopLatchReleasesTheMotorDriverFaultFirst)
{
    makeEStop([]() { return true; });

    e_stop->resetEStopLatch();

    const std::vector<std::pair<PortWrite, bool>> expected = {
        {PortWrite::kMotorDriverFault, false},
        {PortWrite::kLatchReset, true},
    };
    EXPECT_EQ(io->write_log, expected);
}

TEST_F(EmergencyStopTest, ResetEStopLatchWrapsAFailedRelease)
{
    makeEStop([]() { return true; });
    io->throw_on_motor_driver_fault = true;

    EXPECT_THROW(e_stop->resetEStopLatch(), std::runtime_error);
    EXPECT_EQ(io->reset_latch_calls, 0u);
}

TEST_F(EmergencyStopTest, RequestMotorDriverFaultPassesThroughWithoutWriting)
{
    makeEStop([]() { return true; });

    e_stop->requestMotorDriverFault();

    EXPECT_EQ(io->request_motor_driver_fault_calls, 1u);
    EXPECT_TRUE(io->write_log.empty());
}

// --- releaseStartupTriggers(): configure-time release of both software E-Stop inputs ---------

TEST_F(EmergencyStopTest, ReleaseStartupTriggersClearsUserButtonThenMotorDriverFault)
{
    makeEStop([]() { return true; });

    e_stop->releaseStartupTriggers();

    const std::vector<std::pair<PortWrite, bool>> expected = {
        {PortWrite::kUserButton, false},
        {PortWrite::kMotorDriverFault, false},
    };
    EXPECT_EQ(io->write_log, expected);
}

// At configure the zero-velocity flags are still at their fail-safe false, so consulting the
// check would fail every on_configure().
TEST_F(EmergencyStopTest, ReleaseStartupTriggersDoesNotConsultTheZeroVelocityCheck)
{
    unsigned zero_velocity_check_calls = 0;
    makeEStop([&zero_velocity_check_calls]() {
        ++zero_velocity_check_calls;
        return false;
    });

    EXPECT_NO_THROW(e_stop->releaseStartupTriggers());

    EXPECT_EQ(zero_velocity_check_calls, 0u);
    EXPECT_EQ(io->write_log.size(), 2u);
}

TEST_F(EmergencyStopTest, ReleaseStartupTriggersPropagatesUserButtonFailureUnwrapped)
{
    makeEStop([]() { return true; });
    io->throw_on_user_button = true;

    EXPECT_THROW(e_stop->releaseStartupTriggers(), FakeIoError);
    // A failed user-button write skips the motor-driver-fault write.
    EXPECT_TRUE(io->write_log.empty());
}

TEST_F(EmergencyStopTest, ReleaseStartupTriggersPropagatesMotorDriverFaultFailureUnwrapped)
{
    makeEStop([]() { return true; });
    io->throw_on_motor_driver_fault = true;

    EXPECT_THROW(e_stop->releaseStartupTriggers(), FakeIoError);

    const std::vector<std::pair<PortWrite, bool>> expected = {
        {PortWrite::kUserButton, false},
    };
    EXPECT_EQ(io->write_log, expected);
}

}  // namespace isaacbot_amr_hardware_interface
