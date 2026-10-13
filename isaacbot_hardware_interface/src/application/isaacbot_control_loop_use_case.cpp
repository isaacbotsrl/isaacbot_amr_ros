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

#include "isaacbot_hardware_interface/application/isaacbot_control_loop_use_case.hpp"

#include <exception>
#include <mutex>
#include <utility>

namespace isaacbot_hardware_interface
{

IsaacbotControlLoopUseCase::IsaacbotControlLoopUseCase(
    std::shared_ptr<IsaacbotDriverInterface> isaacbot_driver,
    std::shared_ptr<EmergencyStopInterface> e_stop,
    IsaacbotErrorFilter & error_filter,
    const std::chrono::milliseconds contactor_drop_out_tolerance)
: isaacbot_driver_(std::move(isaacbot_driver))
, e_stop_(std::move(e_stop))
, error_filter_(error_filter)
, contactor_monitor_(contactor_drop_out_tolerance)
{

}

void IsaacbotControlLoopUseCase::updateMotorsStateTimeoutStatus()
{
    error_filter_.updateError(
        ErrorsFilterIds::READ_MOTOR_STATES, isaacbot_driver_->isMotorStatesDataTimedOut());
}

void IsaacbotControlLoopUseCase::updateDriverStateTimeoutStatus()
{
    error_filter_.updateError(
        ErrorsFilterIds::READ_DRIVER_STATE, isaacbot_driver_->isDriverStateDataTimedOut());
}

std::optional<WriteOperationResult> IsaacbotControlLoopUseCase::updateFaultFlagStatus()
{
    const bool flag_error = isaacbot_driver_->isFlagError();
    error_filter_.updateError(ErrorsFilterIds::FAULT_FLAG, flag_error);

    if (!flag_error) {
        return std::nullopt;
    }

    return performWriteOperation([this] { isaacbot_driver_->attemptErrorFlagReset(); });
}

void IsaacbotControlLoopUseCase::updateMotorFailsafeTrippedStatus()
{
    error_filter_.updateError(
        ErrorsFilterIds::MOTOR_FAILSAFE_TRIPPED, isaacbot_driver_->isFailsafeTripped());
}

bool IsaacbotControlLoopUseCase::isMotorFailsafeLatched() const
{
    return error_filter_.isError(ErrorsFilterIds::MOTOR_FAILSAFE_TRIPPED);
}

ContactorFault IsaacbotControlLoopUseCase::updateContactorPlausibility(
    const std::chrono::steady_clock::time_point now)
{
    if (!e_stop_) {
        // No E-Stop port configured: there is nothing to cross-check against, and inventing a
        // verdict here would be worse than declining to give one. The fail-safe behaviour for
        // that case already lives in updateEStopActiveState(), which returns "stopped".
        return ContactorFault::kNone;
    }

    return contactor_monitor_.update(
        e_stop_->readEStopLatchState(), e_stop_->readContactorEngagedState(), now);
}

bool IsaacbotControlLoopUseCase::isContactorFaultLatched() const
{
    return contactor_monitor_.isWeldedFaultLatched();
}

bool IsaacbotControlLoopUseCase::isHardwareFaultLatched() const
{
    return isMotorFailsafeLatched() || isContactorFaultLatched();
}

void IsaacbotControlLoopUseCase::resetContactorFault()
{
    contactor_monitor_.reset();
}

const ContactorMonitor & IsaacbotControlLoopUseCase::contactorMonitor() const
{
    return contactor_monitor_;
}

bool IsaacbotControlLoopUseCase::updateEStopActiveState()
{
    if (!e_stop_) {
        return true;
    }

    const bool user_e_stop_triggered = e_stop_->readEStopState();
    const bool e_stop_latched = e_stop_->readEStopLatchState();

    return user_e_stop_triggered || e_stop_latched;
}

bool IsaacbotControlLoopUseCase::shouldCommandMotion(const bool lifecycle_active, const bool e_stop_active)
{
    return lifecycle_active && !e_stop_active;
}

WriteCommandMode IsaacbotControlLoopUseCase::decideWriteCommand(
    const bool lifecycle_active, const bool lifecycle_inactive, const bool e_stop_active,
    const bool hardware_fault_latched)
{
    if (shouldCommandMotion(lifecycle_active, e_stop_active) && !hardware_fault_latched) {
        return WriteCommandMode::kCommandMotion;
    }

    if (lifecycle_active || lifecycle_inactive) {
        return WriteCommandMode::kCommandZero;
    }

    return WriteCommandMode::kSkip;
}

WriteOperationResult IsaacbotControlLoopUseCase::performWriteOperation(
    const std::function<void()> & write_operation)
{
    std::unique_lock<std::mutex> write_lck(write_mtx_, std::defer_lock);

    if (!write_lck.try_lock()) {
        lock_contentions_.fetch_add(1, std::memory_order_relaxed);
        error_filter_.updateError(ErrorsFilterIds::WRITE_CMDS, true);
        return {WriteOperationOutcome::kLockContention, ""};
    }

    try {
        write_operation();
        error_filter_.updateError(ErrorsFilterIds::WRITE_CMDS, false);
        return {WriteOperationOutcome::kSucceeded, ""};
    } catch (const std::exception & e) {
        exceptions_.fetch_add(1, std::memory_order_relaxed);
        // Copies e.what() into error_message on every exception, unlike the old inline
        // RCLCPP_WARN_STREAM_THROTTLE call this replaced, which only paid its string-building
        // cost on ticks the 5s throttle let through - the caller (IsaacbotSystem::
        // logWriteOperationResult()) now owns that throttling instead. Accepted: the exception
        // itself (__cxa_allocate_exception et al.) already costs far more than one std::string
        // copy, so this doesn't change the RT-budget picture on the failure path.
        error_filter_.updateError(ErrorsFilterIds::WRITE_CMDS, true);
        return {WriteOperationOutcome::kExceptionThrown, e.what()};
    }
}

void IsaacbotControlLoopUseCase::recordWriteMode(const WriteCommandMode mode)
{
    switch (mode) {
        case WriteCommandMode::kCommandMotion:
            motion_cycles_.fetch_add(1, std::memory_order_relaxed);
            break;

        case WriteCommandMode::kCommandZero:
            zero_cycles_.fetch_add(1, std::memory_order_relaxed);
            break;

        case WriteCommandMode::kSkip:
            skip_cycles_.fetch_add(1, std::memory_order_relaxed);
            break;
    }
}

WriteCycleStats IsaacbotControlLoopUseCase::writeCycleStats() const
{
    WriteCycleStats s;
    s.motion_cycles = motion_cycles_.load(std::memory_order_relaxed);
    s.zero_cycles = zero_cycles_.load(std::memory_order_relaxed);
    s.skip_cycles = skip_cycles_.load(std::memory_order_relaxed);
    s.lock_contentions = lock_contentions_.load(std::memory_order_relaxed);
    s.exceptions = exceptions_.load(std::memory_order_relaxed);
    return s;
}

}  // namespace isaacbot_hardware_interface
