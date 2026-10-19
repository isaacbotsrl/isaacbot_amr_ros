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

#include "isaacbot_amr_hardware_interface/isaacbot_amr_driver/phidget_driver/phidget_motor_driver.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <limits>
#include <string>

#include "isaacbot_amr_hardware_interface/domain/driver.hpp"
#include "isaacbot_amr_hardware_interface/domain/driver_data_snapshot.hpp"
#include "isaacbot_amr_hardware_interface/isaacbot_amr_driver/phidget_driver/phidget_utils.hpp"

namespace isaacbot_amr_hardware_interface
{

namespace
{

// Steady-clock nanoseconds, the time base of the command-path counters (domain/command_stats.hpp).
// Reading the clock is RT-safe (no syscall on Linux, no allocation).
std::int64_t steadyNowNs()
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

// How long command_worker_ waits on the mailbox before re-checking whether it should stop; also
// the worst-case extra latency of a command whose wakeup was missed (see LatestValueMailbox).
constexpr auto kCommandWorkerWaitTimeout = std::chrono::milliseconds(5);

constexpr std::uint8_t faultBit(const std::size_t bit)
{
    return static_cast<std::uint8_t>(1U << bit);
}

// Current telemetry is reported in units of 10 mA: MotorDriverState::current is an int16, which
// in mA would overflow at 32.7 A - below the DCC1020's 50 A continuous rating. The URDF's
// raw_current_to_amps_scale (0.01) undoes this.
constexpr double kRawCurrentUnitsPerAmp = 100.0;

std::int16_t ampsToRawCurrent(const double amps)
{
    const double raw = std::round(amps * kRawCurrentUnitsPerAmp);
    const double clamped = std::clamp(
        raw, static_cast<double>(std::numeric_limits<std::int16_t>::min()),
        static_cast<double>(std::numeric_limits<std::int16_t>::max()));

    return static_cast<std::int16_t>(clamped);
}

PhidgetHandle asHandle(PhidgetMotorVelocityControllerHandle h)
{
    return reinterpret_cast<PhidgetHandle>(h);
}

}  // namespace

PhidgetDriver::PhidgetDriver()
{

}

std::future<void> PhidgetDriver::initialize()
{
    std::lock_guard<std::mutex> lck(init_mtx_);
    init_promise_ = std::promise<void>();

    std::future<void> future = init_promise_.get_future();

    for (auto & [name, motor_driver] : motor_drivers_) {
        try {
            motor_driver->initialize();
        } catch (const std::runtime_error & e) {
            throw std::runtime_error(
                "Motor driver initilize exception on " +
                motorNamesToString(name) +
                " motor: " +
                std::string(e.what()));
        }
    }

    try {
        init_promise_.set_value();
    } catch (const std::future_error & e) {
        if (e.code() == std::make_error_code(std::future_errc::promise_already_satisfied)) {
            RCLCPP_WARN_STREAM(logger_, "An exception occurred while setting init promise: " << e.what());
        }
    }

    return future;
}

DriverState PhidgetDriver::readState()
{
    DriverState driver_state;

    const auto motor_driver = this->getMotorDriver(MotorNames::DEFAULT);
    const auto motor_state = motor_driver->readState();
    driver_state.fault_flags = motor_driver->getFaultFlags();
    driver_state.runtime_stat_flag = 0;
    driver_state.driver_current = motor_state.current;
    driver_state.temp = motor_state.temp;

    return driver_state;
}

bool PhidgetDriver::isCommunicationError()
{
    return getMotorDriver(MotorNames::DEFAULT)->isCommunicationError();
}

void PhidgetDriver::addMotorDriver(
    const MotorNames name,
    std::shared_ptr<MotorDriverInterface> motor_driver)
{
    if (std::dynamic_pointer_cast<PhidgetMotorDriver>(motor_driver) == nullptr) {
        throw std::runtime_error("Motor driver is not of type PhidgetMotorDriver");
    }

    motor_drivers_.emplace(name, motor_driver);
}

std::shared_ptr<MotorDriverInterface> PhidgetDriver::getMotorDriver(const MotorNames name)
{
    auto it = motor_drivers_.find(name);

    if (it == motor_drivers_.end()) {
        throw std::runtime_error("Motor driver with name '" +
            motorNamesToString(name) +
            "' does not exist");
    }

    return it->second;
}

PhidgetMotorDriver::PhidgetMotorDriver(
    const DrivetrainSettings & drivetrain_settings,
    std::weak_ptr<PhidgetDriver> driver,
    const std::uint8_t channel,
    const std::int32_t serial_number,
    const bool dir_reverse)
: driver_(driver)
, settings_(drivetrain_settings)
, channel_(channel)
, serial_number_(serial_number)
, direction_reversed_(dir_reverse)
, comm_timeout_(std::chrono::milliseconds(drivetrain_settings.driver_comm_timeout_ms))
{
    RCLCPP_INFO(
        logger_,
        "Create phidget motor driver hub port = %d, encoder resolution = %f lines, current limit = %f A",
        channel_, settings_.encoder_resolution, settings_.motor_current_limit);
}

PhidgetMotorDriver::~PhidgetMotorDriver()
{
    RCLCPP_INFO(logger_, "Destroy phidget motor driver hub port = %d", channel_);

    stopCommandWorker();

    // Closing disengages (coasts) the motor; ask the profile to stop it first. Best effort.
    if (motor_handle_ != nullptr) {
        (void)PhidgetMotorVelocityController_setTargetVelocity(motor_handle_, 0.0);
    }

    PhidgetHandle handle = asHandle(motor_handle_);
    closeAndDelete(&handle);

    if (encoder_handle_ != nullptr) {
        handle = reinterpret_cast<PhidgetHandle>(encoder_handle_);
        closeAndDelete(&handle);
    }

    handle = reinterpret_cast<PhidgetHandle>(current_handle_);
    closeAndDelete(&handle);

    handle = reinterpret_cast<PhidgetHandle>(temperature_handle_);
    closeAndDelete(&handle);
}

namespace
{

void throwOnError(const PhidgetReturnCode ret, const std::string & what, const std::uint8_t channel)
{
    if (ret != EPHIDGET_OK) {
        throw std::runtime_error(
            what + " failed for motor hub port " + std::to_string(channel) + ": " +
            phidgetReturnCodeToString(ret));
    }
}

using VelocityControllerGetter = PhidgetReturnCode (*)(PhidgetMotorVelocityControllerHandle, double *);
using VelocityControllerSetter = PhidgetReturnCode (*)(PhidgetMotorVelocityControllerHandle, double);

// Sets a configured value after checking it against the board's own range, so an out-of-range
// URDF value fails configure with its name instead of an anonymous SDK error.
void setInRange(
    PhidgetMotorVelocityControllerHandle handle, const std::uint8_t channel, const std::string & name,
    const double value, VelocityControllerGetter get_min, VelocityControllerGetter get_max,
    VelocityControllerSetter set)
{
    double min_value = 0.0;
    double max_value = 0.0;
    throwOnError(get_min(handle, &min_value), "Reading the minimum " + name, channel);
    throwOnError(get_max(handle, &max_value), "Reading the maximum " + name, channel);

    if (value < min_value || value > max_value) {
        throw std::runtime_error(
            name + " = " + std::to_string(value) + " is outside the board's range [" +
            std::to_string(min_value) + ", " + std::to_string(max_value) + "] on motor hub port " +
            std::to_string(channel));
    }

    throwOnError(set(handle, value), "Setting " + name, channel);
}

}  // namespace

void PhidgetMotorDriver::initialize()
{
    // initialize() is retried by PhidgetIsaacbotDriver on failure; never leave a worker sending on
    // a channel that is about to be re-created.
    stopCommandWorker();

    if (motor_handle_ == nullptr) {
        throwOnError(PhidgetMotorVelocityController_create(&motor_handle_), "Creating the velocity controller", channel_);
    } else {
        (void)Phidget_close(asHandle(motor_handle_));
    }

    // Handlers before open, so nothing reported during attach (e.g. an E-Stop that is already
    // open) is missed.
    throwOnError(
        Phidget_setOnErrorHandler(asHandle(motor_handle_), errorHandler, this),
        "Setting the error handler", channel_);
    throwOnError(
        PhidgetMotorVelocityController_setOnVelocityChangeHandler(motor_handle_, velocityChangeHandler, this),
        "Setting the velocity change handler", channel_);

    openWaitForAttachment(asHandle(motor_handle_), serial_number_, channel_, false, 0);

    if (serial_number_ == -1) {
        throwOnError(
            Phidget_getDeviceSerialNumber(asHandle(motor_handle_), &serial_number_),
            "Reading the serial number", channel_);
    }

    Phidget_DeviceID device_id = PHIDID_NOTHING;

    if (Phidget_getDeviceID(asHandle(motor_handle_), &device_id) == EPHIDGET_OK && device_id != PHIDID_DCC1020)
    {
        RCLCPP_WARN(
            logger_, "Motor hub port %d: attached device ID 0x%x is not a DCC1020 (0x%x).", channel_,
            static_cast<unsigned>(device_id), static_cast<unsigned>(PHIDID_DCC1020));
    }

    configureMotorChannel();

    std::uint32_t velocity_interval_ms = 0;
    (void)PhidgetMotorVelocityController_getDataInterval(motor_handle_, &velocity_interval_ms);

    openEncoderChannel(velocity_interval_ms);

    if (current_handle_ == nullptr) {
        throwOnError(PhidgetCurrentInput_create(&current_handle_), "Creating the current input", channel_);
    } else {
        (void)Phidget_close(reinterpret_cast<PhidgetHandle>(current_handle_));
    }

    throwOnError(
        PhidgetCurrentInput_setOnCurrentChangeHandler(current_handle_, currentChangeHandler, this),
        "Setting the current change handler", channel_);

    openWaitForAttachment(reinterpret_cast<PhidgetHandle>(current_handle_), serial_number_, channel_, false, 0);

    // Force the current channel to report on every data interval rather than only when the
    // reading changes — this callback firing is also used as a communication-liveness signal
    // (see last_update_time_ns_), and a stationary/steady-current robot must not look "stale".
    std::uint32_t current_min_interval = 0;
    throwOnError(
        PhidgetCurrentInput_getMinDataInterval(current_handle_, &current_min_interval),
        "Reading the current input minimum data interval", channel_);
    throwOnError(
        PhidgetCurrentInput_setDataInterval(current_handle_, current_min_interval),
        "Setting the current input data interval", channel_);
    throwOnError(
        PhidgetCurrentInput_setCurrentChangeTrigger(current_handle_, 0.0),
        "Setting the current input change trigger", channel_);

    if (temperature_handle_ == nullptr) {
        throwOnError(
            PhidgetTemperatureSensor_create(&temperature_handle_), "Creating the temperature sensor", channel_);
    } else {
        (void)Phidget_close(reinterpret_cast<PhidgetHandle>(temperature_handle_));
    }

    throwOnError(
        PhidgetTemperatureSensor_setOnTemperatureChangeHandler(temperature_handle_, temperatureChangeHandler, this),
        "Setting the temperature change handler", channel_);

    openWaitForAttachment(reinterpret_cast<PhidgetHandle>(temperature_handle_), serial_number_, channel_, false, 0);

    // Same rationale as the current channel above: force periodic reporting so it can serve as
    // a communication-liveness signal even when the temperature reading is not changing.
    std::uint32_t temperature_min_interval = 0;
    throwOnError(
        PhidgetTemperatureSensor_getMinDataInterval(temperature_handle_, &temperature_min_interval),
        "Reading the temperature sensor minimum data interval", channel_);
    throwOnError(
        PhidgetTemperatureSensor_setDataInterval(temperature_handle_, temperature_min_interval),
        "Setting the temperature sensor data interval", channel_);
    throwOnError(
        PhidgetTemperatureSensor_setTemperatureChangeTrigger(temperature_handle_, 0.0),
        "Setting the temperature sensor change trigger", channel_);

    startCommandWorker();
}

void PhidgetMotorDriver::openEncoderChannel(const std::uint32_t data_interval_ms)
{
    try {
        if (encoder_handle_ == nullptr) {
            throwOnError(PhidgetEncoder_create(&encoder_handle_), "Creating the encoder", channel_);
        } else {
            (void)Phidget_close(reinterpret_cast<PhidgetHandle>(encoder_handle_));
        }

        throwOnError(
            PhidgetEncoder_setOnPositionChangeHandler(encoder_handle_, positionChangeHandler, this),
            "Setting the encoder position change handler", channel_);

        openWaitForAttachment(reinterpret_cast<PhidgetHandle>(encoder_handle_), serial_number_, channel_, false, 0);

        throwOnError(
            PhidgetEncoder_setIOMode(encoder_handle_, ENCODER_IO_MODE_OPEN_COLLECTOR_10K),
            "Setting the encoder IO mode", channel_);

        // Position only needs to be as fresh as the velocity feedback; the encoder could report
        // every 1 ms, which would just be 4 x 1 kHz of callbacks for nothing.
        std::uint32_t min_interval = 0;
        std::uint32_t max_interval = 0;
        throwOnError(
            PhidgetEncoder_getMinDataInterval(encoder_handle_, &min_interval),
            "Reading the encoder minimum data interval", channel_);
        throwOnError(
            PhidgetEncoder_getMaxDataInterval(encoder_handle_, &max_interval),
            "Reading the encoder maximum data interval", channel_);
        throwOnError(
            PhidgetEncoder_setDataInterval(encoder_handle_, std::clamp(data_interval_ms, min_interval, max_interval)),
            "Setting the encoder data interval", channel_);
    } catch (const std::runtime_error & e) {
        RCLCPP_WARN(
            logger_,
            "Motor hub port %d: Encoder channel unavailable (%s); joint position is integrated from "
            "the velocity controller's feedback instead and will drift.",
            channel_, e.what());

        if (encoder_handle_ != nullptr) {
            PhidgetHandle handle = reinterpret_cast<PhidgetHandle>(encoder_handle_);
            closeAndDelete(&handle);
            encoder_handle_ = nullptr;
        }

        position_from_velocity_.store(true, std::memory_order_relaxed);
        return;
    }

    position_from_velocity_.store(false, std::memory_order_relaxed);
}

void PhidgetMotorDriver::configureMotorChannel()
{
    // Everything below except the data interval is in rescaled units, so this goes first.
    const double rescale_factor = motorRpmRescaleFactor(settings_.encoder_resolution);

    if (rescale_factor <= 0.0) {
        throw std::runtime_error("encoder_resolution must be > 0 to rescale the velocity controller.");
    }

    throwOnError(
        PhidgetMotorVelocityController_setRescaleFactor(motor_handle_, rescale_factor),
        "Setting the rescale factor", channel_);

    std::uint32_t min_interval = 0;
    std::uint32_t max_interval = 0;
    throwOnError(
        PhidgetMotorVelocityController_getMinDataInterval(motor_handle_, &min_interval),
        "Reading the minimum data interval", channel_);
    throwOnError(
        PhidgetMotorVelocityController_getMaxDataInterval(motor_handle_, &max_interval),
        "Reading the maximum data interval", channel_);

    const std::uint32_t requested_interval = settings_.motor_data_interval_ms.value_or(min_interval);
    const std::uint32_t data_interval = std::clamp(requested_interval, min_interval, max_interval);

    if (data_interval != requested_interval) {
        RCLCPP_WARN(
            logger_, "Motor hub port %d: motor_data_interval_ms %u clamped to the board's [%u, %u] ms.",
            channel_, requested_interval, min_interval, max_interval);
    }

    throwOnError(
        PhidgetMotorVelocityController_setDataInterval(motor_handle_, data_interval),
        "Setting the data interval", channel_);

    std::uint32_t actual_interval = data_interval;
    (void)PhidgetMotorVelocityController_getDataInterval(motor_handle_, &actual_interval);
    velocity_stale_timeout_ns_.store(encoderStaleTimeout(actual_interval).count(), std::memory_order_relaxed);

    setInRange(
        motor_handle_, channel_, "motor_current_limit", settings_.motor_current_limit,
        PhidgetMotorVelocityController_getMinCurrentLimit, PhidgetMotorVelocityController_getMaxCurrentLimit,
        PhidgetMotorVelocityController_setCurrentLimit);

    if (settings_.motor_surge_current_limit) {
        setInRange(
            motor_handle_, channel_, "motor_surge_current_limit", *settings_.motor_surge_current_limit,
            PhidgetMotorVelocityController_getMinSurgeCurrentLimit,
            PhidgetMotorVelocityController_getMaxSurgeCurrentLimit,
            PhidgetMotorVelocityController_setSurgeCurrentLimit);
    }

    if (settings_.motor_acceleration_rpm_s) {
        setInRange(
            motor_handle_, channel_, "motor_acceleration_rpm_s", *settings_.motor_acceleration_rpm_s,
            PhidgetMotorVelocityController_getMinAcceleration, PhidgetMotorVelocityController_getMaxAcceleration,
            PhidgetMotorVelocityController_setAcceleration);
    }

    if (settings_.motor_stall_velocity_rpm) {
        setInRange(
            motor_handle_, channel_, "motor_stall_velocity_rpm", *settings_.motor_stall_velocity_rpm,
            PhidgetMotorVelocityController_getMinStallVelocity, PhidgetMotorVelocityController_getMaxStallVelocity,
            PhidgetMotorVelocityController_setStallVelocity);
    }

    if (settings_.motor_inductance) {
        setInRange(
            motor_handle_, channel_, "motor_inductance", *settings_.motor_inductance,
            PhidgetMotorVelocityController_getMinInductance, PhidgetMotorVelocityController_getMaxInductance,
            PhidgetMotorVelocityController_setInductance);
    }

    // The SDK publishes no range for the gains, the dead band or the failsafe current limit.
    if (settings_.velocity_kp) {
        throwOnError(PhidgetMotorVelocityController_setKp(motor_handle_, *settings_.velocity_kp), "Setting Kp", channel_);
    }

    if (settings_.velocity_ki) {
        throwOnError(PhidgetMotorVelocityController_setKi(motor_handle_, *settings_.velocity_ki), "Setting Ki", channel_);
    }

    if (settings_.velocity_kd) {
        throwOnError(PhidgetMotorVelocityController_setKd(motor_handle_, *settings_.velocity_kd), "Setting Kd", channel_);
    }

    if (settings_.velocity_deadband_rpm) {
        throwOnError(
            PhidgetMotorVelocityController_setDeadBand(motor_handle_, *settings_.velocity_deadband_rpm),
            "Setting the dead band", channel_);
    }

    throwOnError(
        PhidgetMotorVelocityController_setFailsafeBrakingEnabled(
            motor_handle_, settings_.motor_failsafe_braking_enabled ? 1 : 0),
        "Setting failsafe braking", channel_);

    if (settings_.motor_failsafe_current_limit) {
        throwOnError(
            PhidgetMotorVelocityController_setFailsafeCurrentLimit(motor_handle_, *settings_.motor_failsafe_current_limit),
            "Setting the failsafe current limit", channel_);
    }

    double max_target_velocity = 0.0;
    throwOnError(
        PhidgetMotorVelocityController_getMaxTargetVelocity(motor_handle_, &max_target_velocity),
        "Reading the maximum target velocity", channel_);

    if (static_cast<double>(settings_.max_rpm_motor_speed) > max_target_velocity) {
        throw std::runtime_error(
            "max_rpm_motor_speed = " + std::to_string(settings_.max_rpm_motor_speed) +
            " RPM exceeds the board's maximum target velocity " + std::to_string(max_target_velocity) +
            " RPM on motor hub port " + std::to_string(channel_));
    }

    // Log what the board actually runs with - for the unset gains these are its defaults, the
    // starting point for tuning.
    double kp = 0.0, ki = 0.0, kd = 0.0, dead_band = 0.0, acceleration = 0.0;
    double current_limit = 0.0, surge_current_limit = 0.0, stall_velocity = 0.0, inductance = 0.0;
    (void)PhidgetMotorVelocityController_getKp(motor_handle_, &kp);
    (void)PhidgetMotorVelocityController_getKi(motor_handle_, &ki);
    (void)PhidgetMotorVelocityController_getKd(motor_handle_, &kd);
    (void)PhidgetMotorVelocityController_getDeadBand(motor_handle_, &dead_band);
    (void)PhidgetMotorVelocityController_getAcceleration(motor_handle_, &acceleration);
    (void)PhidgetMotorVelocityController_getCurrentLimit(motor_handle_, &current_limit);
    (void)PhidgetMotorVelocityController_getSurgeCurrentLimit(motor_handle_, &surge_current_limit);
    (void)PhidgetMotorVelocityController_getStallVelocity(motor_handle_, &stall_velocity);
    (void)PhidgetMotorVelocityController_getInductance(motor_handle_, &inductance);

    RCLCPP_INFO(
        logger_,
        "Motor hub port %d (serial %d): Kp=%g%s Ki=%g%s Kd=%g%s dead band=%g RPM, acceleration=%g RPM/s, "
        "current limit=%g A, surge=%g A, stall velocity=%g RPM, inductance=%g, data interval=%u ms, "
        "rescale=%g, failsafe braking=%s",
        channel_, serial_number_, kp, settings_.velocity_kp ? "" : " (board default)", ki,
        settings_.velocity_ki ? "" : " (board default)", kd, settings_.velocity_kd ? "" : " (board default)",
        dead_band, acceleration, current_limit, surge_current_limit, stall_velocity, inductance, actual_interval,
        rescale_factor, settings_.motor_failsafe_braking_enabled ? "on" : "off");
}

void PhidgetMotorDriver::reopenMotorChannel()
{
    const PhidgetHandle handle = asHandle(motor_handle_);

    // Cleared BEFORE the close: the error handler stays registered, so whatever the board still
    // reports while re-attaching (e.g. an E-Stop input that is still open) must survive.
    fault_bits_.store(0, std::memory_order_relaxed);
    // Don't integrate the gap across the re-open into the fallback position.
    last_velocity_event_for_integration_ns_.store(0, std::memory_order_relaxed);

    // Closing also disables the failsafe and disengages the motor - neither can be undone any
    // other way once tripped.
    (void)Phidget_close(handle);
    failsafe_enabled_ = false;

    // Same addressing as initialize(), with the serial number it resolved, so we re-attach to the
    // very same board. The handlers set in initialize() stay registered on the handle.
    openWaitForAttachment(handle, serial_number_, channel_, false, 0);

    configureMotorChannel();
}

void PhidgetMotorDriver::enableFailsafe()
{
    std::uint32_t min_time = 0;
    std::uint32_t max_time = 0;
    throwOnError(
        PhidgetMotorVelocityController_getMinFailsafeTime(motor_handle_, &min_time),
        "Reading the minimum failsafe time", channel_);
    throwOnError(
        PhidgetMotorVelocityController_getMaxFailsafeTime(motor_handle_, &max_time),
        "Reading the maximum failsafe time", channel_);

    if (settings_.motor_failsafe_timeout_ms < min_time || settings_.motor_failsafe_timeout_ms > max_time) {
        throw std::runtime_error(
            "motor_failsafe_timeout_ms = " + std::to_string(settings_.motor_failsafe_timeout_ms) +
            " is outside the board's range [" + std::to_string(min_time) + ", " + std::to_string(max_time) +
            "] ms on motor hub port " + std::to_string(channel_));
    }

    const PhidgetReturnCode ret =
        PhidgetMotorVelocityController_enableFailsafe(motor_handle_, settings_.motor_failsafe_timeout_ms);

    if (ret != EPHIDGET_OK) {
        throw std::runtime_error(
            "Failed to arm fail safe (timeout " + std::to_string(settings_.motor_failsafe_timeout_ms) +
            "ms) for motor hub port " + std::to_string(channel_) + ": " + returnCodeToString(ret));
    }

    failsafe_enabled_ = true;
}

void PhidgetMotorDriver::setEngaged(const bool engaged)
{
    throwOnError(
        PhidgetMotorVelocityController_setEngaged(motor_handle_, engaged ? 1 : 0),
        engaged ? "Engaging the motor" : "Disengaging the motor", channel_);
}

std::string PhidgetMotorDriver::returnCodeToString(const PhidgetReturnCode ret)
{
    return phidgetReturnCodeToString(ret);
}

MotorDriverState PhidgetMotorDriver::readState()
{
    // Non-blocking: never stall the RT control loop waiting on the SDK callback thread.
    // On contention, just return the last successfully-read snapshot.
    std::unique_lock<std::mutex> lck(state_mtx_, std::try_to_lock);

    if (lck.owns_lock()) {
        state_snapshot_ = state_;
    }

    const auto last_velocity_event_ns = last_velocity_event_ns_.load(std::memory_order_relaxed);

    if (last_velocity_event_ns == 0 ||
        isCommTimedOut(
            std::chrono::steady_clock::time_point(std::chrono::nanoseconds(last_velocity_event_ns)),
            std::chrono::steady_clock::now(),
            std::chrono::nanoseconds(velocity_stale_timeout_ns_.load(std::memory_order_relaxed))))
    {
        state_snapshot_.vel = 0.0;
    }

    return state_snapshot_;
}

std::chrono::nanoseconds PhidgetMotorDriver::encoderStaleTimeout(
    const std::uint32_t data_interval_ms)
{
    return std::max<std::chrono::nanoseconds>(
        std::chrono::milliseconds(3u * data_interval_ms), kMinVelocityStaleTimeout);
}

double PhidgetMotorDriver::motorRpmToLines(const double rpm, const double dt_s, const float lines)
{
    if (dt_s <= 0.0 || lines <= 0.0f) {
        return 0.0;
    }

    return rpm / 60.0 * dt_s * static_cast<double>(lines);
}

bool PhidgetMotorDriver::isCommunicationError()
{
    const auto last_update_ns = last_update_time_ns_.load(std::memory_order_relaxed);

    if (last_update_ns == 0) {
        // No telemetry callback has fired yet since construction - treat as a comm error.
        return true;
    }

    const auto last_update = std::chrono::steady_clock::time_point(
        std::chrono::nanoseconds(last_update_ns));

    return isCommTimedOut(last_update, std::chrono::steady_clock::now(), comm_timeout_);
}

bool PhidgetMotorDriver::isCommTimedOut(
    const std::chrono::steady_clock::time_point & last_update,
    const std::chrono::steady_clock::time_point & now,
    const std::chrono::nanoseconds & timeout)
{
    return (now - last_update) > timeout;
}

void CCONV PhidgetMotorDriver::velocityChangeHandler(
    PhidgetMotorVelocityControllerHandle phid,
    void * ctx,
    double velocity)
{
    (void)phid;

    PhidgetMotorDriver * driver = static_cast<PhidgetMotorDriver*>(ctx);

    const std::int64_t now_ns = steadyNowNs();

    driver->last_update_time_ns_.store(now_ns, std::memory_order_relaxed);

    // Rescaled to motor RPM in configureMotorChannel().
    const double rpm = driver->direction_reversed_ ? -velocity : velocity;

    {
        std::lock_guard<std::mutex> lck(driver->state_mtx_);
        driver->state_.vel = rpm;

        if (driver->position_from_velocity_.load(std::memory_order_relaxed)) {
            const std::int64_t previous_ns =
                driver->last_velocity_event_for_integration_ns_.exchange(now_ns, std::memory_order_relaxed);

            if (previous_ns != 0) {
                const double dt_s = static_cast<double>(now_ns - previous_ns) * 1e-9;
                driver->integrated_lines_ += motorRpmToLines(rpm, dt_s, driver->settings_.encoder_resolution);
            }

            driver->state_.pos = static_cast<std::int64_t>(std::llround(driver->integrated_lines_));
        }
    }

    driver->last_velocity_event_ns_.store(now_ns, std::memory_order_relaxed);
}

void CCONV PhidgetMotorDriver::positionChangeHandler(
    PhidgetEncoderHandle phid,
    void *ctx,
    int positionChange,
    double timeChange,
    int indexTriggered)
{
    (void)positionChange;
    (void)timeChange;
    (void)indexTriggered;

    PhidgetMotorDriver * driver = static_cast<PhidgetMotorDriver*>(ctx);

    driver->last_update_time_ns_.store(steadyNowNs(), std::memory_order_relaxed);

    std::int64_t position = 0;

    if (PhidgetEncoder_getPosition(phid, &position) == EPHIDGET_OK) {
        const std::int64_t counts = driver->direction_reversed_ ? -position : position;

        // MotorDriverState::pos is in encoder lines (4 quadrature counts each).
        std::lock_guard<std::mutex> lck(driver->state_mtx_);
        driver->state_.pos = counts / 4;
    }
}

void CCONV PhidgetMotorDriver::currentChangeHandler(
    PhidgetCurrentInputHandle phid,
    void * ctx,
    double current
)
{
    (void)phid;

    PhidgetMotorDriver * driver = static_cast<PhidgetMotorDriver*>(ctx);

    driver->last_update_time_ns_.store(steadyNowNs(), std::memory_order_relaxed);

    std::lock_guard<std::mutex> lck(driver->state_mtx_);
    driver->state_.current = ampsToRawCurrent(current);
}

void CCONV PhidgetMotorDriver::temperatureChangeHandler(
    PhidgetTemperatureSensorHandle phid,
    void *ctx,
    double temperature
)
{
    (void)phid;

    PhidgetMotorDriver * driver = static_cast<PhidgetMotorDriver*>(ctx);

    driver->last_update_time_ns_.store(steadyNowNs(), std::memory_order_relaxed);

    std::lock_guard<std::mutex> lck(driver->state_mtx_);
    driver->state_.temp = static_cast<float>(temperature);
}

void CCONV PhidgetMotorDriver::errorHandler(
    PhidgetHandle phid,
    void * ctx,
    Phidget_ErrorEventCode code,
    const char * description)
{
    (void)phid;

    PhidgetMotorDriver * driver = static_cast<PhidgetMotorDriver*>(ctx);

    const ErrorEventEffect effect = errorEventEffect(code);

    if (effect.set_fault_bits != 0) {
        driver->fault_bits_.fetch_or(effect.set_fault_bits, std::memory_order_relaxed);
    }

    if (effect.clear_fault_bits != 0) {
        driver->fault_bits_.fetch_and(
            static_cast<std::uint8_t>(~effect.clear_fault_bits), std::memory_order_relaxed);
    }

    if (effect.latches_failsafe) {
        driver->failsafe_tripped_.store(true, std::memory_order_relaxed);
    }

    // SDK callback thread, not the RT thread - logging is fine here, and these are rare.
    if (code == EEPHIDGET_OK) {
        RCLCPP_INFO(
            driver->logger_, "Motor hub port %d: error cleared (%s)", driver->channel_,
            description != nullptr ? description : "");
    } else {
        RCLCPP_ERROR(
            driver->logger_, "Motor hub port %d: error event 0x%x: %s%s", driver->channel_,
            static_cast<unsigned>(code), description != nullptr ? description : "",
            effect.latches_failsafe ? " - motor disengaged, latched until the E-Stop latch is reset" : "");
    }
}

PhidgetMotorDriver::ErrorEventEffect PhidgetMotorDriver::errorEventEffect(const Phidget_ErrorEventCode code)
{
    ErrorEventEffect effect;

    switch (code) {
        case EEPHIDGET_OK:
            effect.clear_fault_bits = transientFaultBits();
            break;

        case EEPHIDGET_OVERTEMP:
            effect.set_fault_bits = faultBit(FaultFlag::kOverTemperatureBit);
            break;

        case EEPHIDGET_OVERCURRENT:
            effect.set_fault_bits = faultBit(FaultFlag::kOverCurrentBit);
            break;

        case EEPHIDGET_OVERVOLTAGE:
            effect.set_fault_bits = faultBit(FaultFlag::kOverVoltageBit);
            break;

        case EEPHIDGET_BADPOWER:
        case EEPHIDGET_VOLTAGEERROR:
            effect.set_fault_bits = faultBit(FaultFlag::kBadPowerBit);
            break;

        // The controller freewheels until power-cycled (DCC1020: supply >= 42 V).
        case EEPHIDGET_ENERGYDUMP:
            effect.set_fault_bits = faultBit(FaultFlag::kOverVoltageBit);
            effect.latches_failsafe = true;
            break;

        // StallVelocity protection disengaged the motor.
        case EEPHIDGET_MOTORSTALL:
            effect.set_fault_bits = faultBit(FaultFlag::kMotorStallBit);
            effect.latches_failsafe = true;
            break;

        // The board's E-Stop input opened; it rejects commands until the channel is re-opened.
        case EEPHIDGET_ESTOP:
            effect.set_fault_bits = faultBit(FaultFlag::kEmergencyStopBit);
            effect.latches_failsafe = true;
            break;

        // The watchdog tripped - reported here even when no command was in flight to see it.
        case EEPHIDGET_FAILSAFE:
            effect.latches_failsafe = true;
            break;

        default:
            break;
    }

    return effect;
}

std::uint8_t PhidgetMotorDriver::transientFaultBits()
{
    return faultBit(FaultFlag::kOverTemperatureBit) | faultBit(FaultFlag::kOverCurrentBit) |
           faultBit(FaultFlag::kOverVoltageBit) | faultBit(FaultFlag::kBadPowerBit);
}

std::uint8_t PhidgetMotorDriver::getFaultFlags() const
{
    return fault_bits_.load(std::memory_order_relaxed);
}

bool PhidgetMotorDriver::isFailsafeTrippedReturnCode(const PhidgetReturnCode res)
{
    return res == EPHIDGET_FAILSAFE;
}

PhidgetMotorDriver::FailsafeAction PhidgetMotorDriver::selectFailsafeAction(
    const bool configured, const bool enabled, const bool tripped)
{
    if (!configured) {
        return FailsafeAction::kNone;
    }

    if (tripped) {
        return FailsafeAction::kReopenAndEnable;
    }

    return enabled ? FailsafeAction::kFeed : FailsafeAction::kEnable;
}

void PhidgetMotorDriver::armFailsafe()
{
    std::lock_guard<std::mutex> lck(failsafe_mtx_);

    // Taken and cleared up front, not after engaging: an error event delivered while this runs
    // (an E-Stop reported on re-attach, a stall right after engage) must leave the latch set.
    const bool tripped = failsafe_tripped_.exchange(false, std::memory_order_relaxed);

    try {
        armAndEngage(tripped);
    } catch (...) {
        // Still not recovered - keep motion refused.
        failsafe_tripped_.store(true, std::memory_order_relaxed);
        throw;
    }
}

void PhidgetMotorDriver::armAndEngage(const bool tripped)
{
    switch (selectFailsafeAction(settings_.motor_failsafe_enabled, failsafe_enabled_, tripped)) {
        case FailsafeAction::kNone:
            // Configured off: never enable or feed. A disengaging fault (stall, E-Stop) still
            // needs the re-open to get the motor back, watchdog or not.
            if (tripped) {
                reopenMotorChannel();
            }
            break;

        case FailsafeAction::kEnable:
            enableFailsafe();
            break;

        case FailsafeAction::kFeed: {
            // Already armed (e.g. on_activate() after a deactivate): enabling again isn't allowed
            // on an open channel, so just feed the timer. If that fails the channel has most
            // likely tripped without a command noticing yet - recover it the same way.
            const PhidgetReturnCode ret = PhidgetMotorVelocityController_resetFailsafe(motor_handle_);

            if (ret != EPHIDGET_OK) {
                RCLCPP_WARN_STREAM(
                    logger_, "PhidgetMotorVelocityController_resetFailsafe() returned " << returnCodeToString(ret)
                        << " for motor hub port " << static_cast<int>(channel_)
                        << "; re-opening the channel and re-arming failsafe.");
                reopenMotorChannel();
                enableFailsafe();
            }
            break;
        }

        case FailsafeAction::kReopenAndEnable:
            reopenMotorChannel();
            enableFailsafe();
            break;
    }

    // Synchronously, not through the worker: the board may still hold the target from before a
    // deactivate or trip, and engaging would drive straight to it. Non-RT context, so blocking
    // is fine here.
    throwOnError(
        PhidgetMotorVelocityController_setTargetVelocity(motor_handle_, 0.0), "Zeroing the target", channel_);

    // The velocity controller does nothing until engaged, and closing / tripping disengages it.
    setEngaged(true);
}

void PhidgetMotorDriver::resetFailsafe()
{
    // A tripped Phidget channel rejects every further call - resetFailsafe() and enableFailsafe()
    // included - until it is closed and re-opened, so the only way to un-stick it is
    // armFailsafe()'s re-open path. For a healthy channel, armFailsafe() just feeds the timer (or
    // enables it, if it never was). Throws std::runtime_error on failure, in which case
    // failsafe_tripped_ stays set and the caller keeps motion refused.
    armFailsafe();
}

bool PhidgetMotorDriver::isFailsafeTripped()
{
    return failsafe_tripped_.load(std::memory_order_relaxed);
}

void PhidgetMotorDriver::sendCmdVel(const float cmd)
{
    if (driver_.expired()) {
        command_stats_.onDroppedNoDriver();
        return;
    }

    command_mailbox_.post(motorTargetRpm(cmd, settings_.max_rpm_motor_speed, direction_reversed_));
}

float PhidgetMotorDriver::motorTargetRpm(const float cmd, const float max_rpm, const bool reversed)
{
    // A NaN/inf would be rejected by the SDK, leaving the board on its previous target (and its
    // watchdog unfed) - stop the wheel instead.
    const float finite = std::isfinite(cmd) ? cmd : 0.0f;
    const float clamped = std::clamp(finite, -max_rpm, max_rpm);

    return reversed ? -clamped : clamped;
}

void PhidgetMotorDriver::startCommandWorker()
{
    if (command_worker_.joinable()) {
        return;
    }

    command_mailbox_.restart();
    command_worker_ = std::thread(&PhidgetMotorDriver::commandWorkerLoop, this);
}

void PhidgetMotorDriver::stopCommandWorker()
{
    if (!command_worker_.joinable()) {
        return;
    }

    command_mailbox_.stop();
    command_worker_.join();
}

void PhidgetMotorDriver::commandWorkerLoop()
{
    // take() returns nullopt both on a plain timeout and after stop(); isStopped() tells them
    // apart.
    while (true) {
        const auto taken = command_mailbox_.take(kCommandWorkerWaitTimeout);

        if (!taken) {
            if (command_mailbox_.isStopped()) {
                return;
            }

            continue;
        }

        command_stats_.onDroppedPending(taken->coalesced);

        command_stats_.onSubmit(steadyNowNs());
        command_in_flight_.store(true, std::memory_order_relaxed);

        // Blocking round trip to the board. Any non-get call also feeds the failsafe timer, so a
        // stalled RT loop (no new targets posted) still lets the watchdog trip.
        const PhidgetReturnCode res =
            PhidgetMotorVelocityController_setTargetVelocity(motor_handle_, static_cast<double>(taken->value));

        command_in_flight_.store(false, std::memory_order_relaxed);

        if (res == EPHIDGET_OK) {
            command_stats_.onCompletion(CommandCompletion::kOk, 0, steadyNowNs());
        } else if (isFailsafeTrippedReturnCode(res)) {
            command_stats_.onCompletion(CommandCompletion::kFailsafe, static_cast<std::int32_t>(res), steadyNowNs());
            failsafe_tripped_.store(true, std::memory_order_relaxed);
        } else {
            command_stats_.onCompletion(CommandCompletion::kError, static_cast<std::int32_t>(res), steadyNowNs());
        }
    }
}

MotorCommandStats PhidgetMotorDriver::getCommandStats() const
{
    return command_stats_.snapshot(command_in_flight_.load(std::memory_order_relaxed), steadyNowNs());
}

}  // namespace isaacbot_amr_hardware_interface
