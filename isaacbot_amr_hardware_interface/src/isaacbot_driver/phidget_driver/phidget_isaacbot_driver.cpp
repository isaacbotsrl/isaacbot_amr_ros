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

#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

#include "isaacbot_amr_hardware_interface/isaacbot_driver/phidget_driver/phidget_isaacbot_driver.hpp"
#include "isaacbot_amr_hardware_interface/isaacbot_driver/phidget_driver/phidget_utils.hpp"

namespace isaacbot_amr_hardware_interface {

PhidgetIsaacbotDriver::PhidgetIsaacbotDriver(
    const DrivetrainSettings & drivetrain_settings,
    const std::chrono::milliseconds activate_wait_time)
: drivetrain_settings_(drivetrain_settings)
, phidget_vel_cmd_converter_(drivetrain_settings)
, activate_wait_time_(activate_wait_time)
{

}

PhidgetIsaacbotDriver::~PhidgetIsaacbotDriver()
{
    drivers_.clear();
    deinitialize();
}

void PhidgetIsaacbotDriver::initialize()
{
    if (initialized_) {
        return;
    }

    try {
        defineDrivers();

        for (auto & [name, driver] : drivers_) {
            data_.emplace(name, DriverDataSnapshot(drivetrain_settings_));
            last_known_data_.emplace(name, data_.at(name));
        }

        initDrivers();

    } catch (const std::runtime_error & e) {
        throw std::runtime_error("Failed to initialize robot driver: " + std::string(e.what()));
    }

    initialized_ = true;
}

void PhidgetIsaacbotDriver::deinitialize()
{
    drivers_.clear();
    initialized_ = false;
}

void PhidgetIsaacbotDriver::activate()
{
    for (auto & [name, driver] : drivers_) {
        try {
            driver->getMotorDriver(MotorNames::DEFAULT)->sendCmdVel(0);
        } catch (const std::runtime_error & e) {
            throw std::runtime_error(
                "Send command exception on " + driverNamesToString(name) +
                "driver : " + std::string(e.what()));
        }
    }

    std::this_thread::sleep_for(activate_wait_time_);
}

void PhidgetIsaacbotDriver::updateCommunicationStatus()
{
    bool has_comm_error = false;

    for (auto & [name, driver] : drivers_) {
        if (driver->isCommunicationError()) {
            has_comm_error = true;
            break;
        }
    }

    has_communication_error_ = has_comm_error;
}

void PhidgetIsaacbotDriver::updateMotorsState()
{
    for (auto & [name, driver] : drivers_) {
        auto motor_driver = driver->getMotorDriver(MotorNames::DEFAULT);
        const auto state = motor_driver->readState();
        const bool data_timed_out = motor_driver->isCommunicationError();

        // Non-blocking: this runs on the RT thread (IsaacbotSystem::read()), and data_mtx_ is also
        // taken by getData() from the diagnostics executor thread - std::mutex has no priority
        // inheritance, so a blocking lock here could stall the RT loop behind that thread. On
        // contention, skip this driver's update; the next cycle refreshes it (readState() is a
        // cached snapshot, so only one tick of freshness is lost). Mirrors getData() below.
        std::unique_lock<std::mutex> lck(data_mtx_, std::try_to_lock);
        if (!lck.owns_lock()) {
            continue;
        }

        setMotorsStates(data_.at(name), state, data_timed_out);
    }
}

void PhidgetIsaacbotDriver::updateDriversState()
{
    for (auto & [name, driver] : drivers_) {
        const auto state = driver->readState();
        const bool data_timed_out = driver->isCommunicationError();

        // Non-blocking, same RT rationale as updateMotorsState() above.
        std::unique_lock<std::mutex> lck(data_mtx_, std::try_to_lock);
        if (!lck.owns_lock()) {
            continue;
        }

        setDriverState(data_.at(name), state, data_timed_out);
    }
}

void PhidgetIsaacbotDriver::attemptErrorFlagReset()
{

}

bool PhidgetIsaacbotDriver::isCommunicationError()
{
    return has_communication_error_;
}

bool PhidgetIsaacbotDriver::isMotorStatesDataTimedOut()
{
    for (const auto & [name, driver_data] : data_) {
        if (driver_data.isMotorStatesDataTimedOut()) {
            return true;
        }
    }

    return false;
}

bool PhidgetIsaacbotDriver::isDriverStateDataTimedOut()
{
    for (const auto & [name, driver_data] : data_) {
        if (driver_data.isDriverStateDataTimedOut()) {
            return true;
        }
    }

    return false;
}

bool PhidgetIsaacbotDriver::isFlagError()
{
    for (const auto & [name, driver_data] : data_) {
        if (driver_data.isFlagError()) {
            return true;
        }
    }

    return false;
}

void PhidgetIsaacbotDriver::armFailsafe()
{
    for (auto & [name, driver] : drivers_) {
        try {
            driver->getMotorDriver(MotorNames::DEFAULT)->armFailsafe();
        } catch (const std::runtime_error & e) {
            throw std::runtime_error(
                "Arm failsafe exception on " + driverNamesToString(name) +
                " driver: " + std::string(e.what()));
        }
    }
}

void PhidgetIsaacbotDriver::resetFailsafe()
{
    for (auto & [name, driver] : drivers_) {
        try {
            driver->getMotorDriver(MotorNames::DEFAULT)->resetFailsafe();
        } catch (const std::runtime_error & e) {
            throw std::runtime_error(
                "Reset failsafe exception on " + driverNamesToString(name) +
                " driver: " + std::string(e.what()));
        }
    }
}

bool PhidgetIsaacbotDriver::isFailsafeTripped()
{
    for (auto & [name, driver] : drivers_) {
        if (driver->getMotorDriver(MotorNames::DEFAULT)->isFailsafeTripped()) {
            return true;
        }
    }

    return false;
}

std::string PhidgetIsaacbotDriver::describeReturnCode(const std::int32_t code) const
{
    return phidgetReturnCodeToString(static_cast<PhidgetReturnCode>(code));
}

MotorCommandStats PhidgetIsaacbotDriver::getCommandStats(const DriverNames name)
{
    // drivers_ is only mutated by initialize()/deinitialize(), never while diagnostics run, and
    // the counters themselves are lock-free atomics. An unknown name reads as all-zero rather than
    // throwing: this is instrumentation and must never disturb its caller.
    const auto it = drivers_.find(name);

    if (it == drivers_.end() || !it->second) {
        return {};
    }

    const auto motor_driver = it->second->getMotorDriver(MotorNames::DEFAULT);

    return motor_driver ? motor_driver->getCommandStats() : MotorCommandStats{};
}

DriverDataSnapshot PhidgetIsaacbotDriver::getData(const DriverNames name)
{
    // `last_known_data_` mirrors `data_`'s keys 1:1 (both seeded together in initialize(), never
    // modified structurally afterwards), so this check doesn't need data_mtx_.
    if (last_known_data_.find(name) == last_known_data_.end()) {
        throw std::runtime_error("Data with name '" + driverNamesToString(name) + "' does not exist.");
    }

    // Non-blocking: this may run on the RT thread (via ArdennesSystem::updateHwStates()), which
    // must never block waiting on the diagnostics thread's own getData() call. On contention,
    // return the last successfully-read snapshot instead - see last_known_data_'s comment.
    std::unique_lock<std::mutex> lck(data_mtx_, std::try_to_lock);
    if (lck.owns_lock()) {
        last_known_data_.at(name) = data_.at(name);
    }

    return last_known_data_.at(name);
}

PhidgetVelocityCommandDataTransformer & PhidgetIsaacbotDriver::getCmdVelConverter()
{
    return phidget_vel_cmd_converter_;
}

void PhidgetIsaacbotDriver::initDrivers()
{
    for (auto & [name, driver] : drivers_) {
        const auto name_str = driverNamesToString(name);

        try {
            auto driver_future = driver->initialize();
            auto driver_status = driver_future.wait_for(std::chrono::seconds(1));

            if (driver_status == std::future_status::ready) {
                try {
                    driver_future.get();
                } catch (const std::exception & e) {
                    throw std::runtime_error(
                        "Init driver for " + name_str + " driver failed with exception: " + std::string(e.what()));
                }
            } else {
                throw std::runtime_error("Init driver for " + name_str + " driver timed out or failed.");
            }
        } catch (const std::runtime_error & e) {
            throw std::runtime_error("Send command exception on " + driverNamesToString(name) + " driver: " + std::string(e.what()));
        }
    }
}

void PhidgetIsaacbotDriver::setMotorsStates(
    DriverDataSnapshot & data,
    const MotorDriverState & state,
    const bool data_timed_out)
{
    data.setMotorsStates(state, data_timed_out);
}

void PhidgetIsaacbotDriver::setDriverState(
    DriverDataSnapshot & data,
    const DriverState & state,
    const bool data_timed_out)
{
    data.setDriverState(state, data_timed_out);
}

}  // namespace isaacbot_amr_hardware_interface
