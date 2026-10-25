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

#ifndef ISAACBOT_AMR_HARDWARE_INTERFACE_ISAACBOT_AMR_DRIVER_PHIDGET_MOTOR_DRIVER_PHIDGET_MOTOR_DRIVER_HPP_
#define ISAACBOT_AMR_HARDWARE_INTERFACE_ISAACBOT_AMR_DRIVER_PHIDGET_MOTOR_DRIVER_PHIDGET_MOTOR_DRIVER_HPP_

#include <phidget22.h>

#include <functional>
#include <vector>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>

#include "isaacbot_amr_hardware_interface/domain/driver.hpp"
#include "isaacbot_amr_hardware_interface/isaacbot_amr_driver/latest_value_mailbox.hpp"
#include "isaacbot_amr_hardware_interface/utils.hpp"

namespace isaacbot_amr_hardware_interface
{

class PhidgetDriver : public DriverInterface
{

public:

    PhidgetDriver();

    std::future<void> initialize() override;

    DriverState readState() override;

    void addMotorDriver(const MotorNames name, std::shared_ptr<MotorDriverInterface> motor_driver) override;

    std::shared_ptr<MotorDriverInterface> getMotorDriver(const MotorNames name) override;

    bool isCommunicationError() override;

private:

    std::mutex init_mtx_;
    std::promise<void> init_promise_;

    std::unordered_map<MotorNames, std::shared_ptr<MotorDriverInterface>> motor_drivers_;

    rclcpp::Logger logger_{rclcpp::get_logger("PhidgetDriver")};
};

// One DCC1020 board (one brushed motor) driven through its on-board MotorVelocityController:
// the board runs the trapezoidal profile and the velocity PID, the host only sends a motor-RPM
// target. Opens, all on channel 0 of the board's VINT hub port:
//   - PhidgetMotorVelocityController: commands, velocity feedback, error events, failsafe
//   - PhidgetEncoder: raw counts for joint position (falls back to integrating the velocity
//     feedback if the board won't open it next to the controller)
//   - PhidgetCurrentInput, PhidgetTemperatureSensor: telemetry and comm liveness
class PhidgetMotorDriver : public MotorDriverInterface
{

public:

    PhidgetMotorDriver(
        const DrivetrainSettings & drivetrain_settings,
        std::weak_ptr<PhidgetDriver> driver,
        const std::uint8_t channel,
        const std::int32_t serial_number,
        const bool dir_reverse);

    ~PhidgetMotorDriver();

    void initialize() override;

    MotorDriverState readState() override;

    // RT-safe: posts the target to command_worker_, which makes the blocking SDK call (the
    // velocity controller has no setTargetVelocity_async).
    void sendCmdVel(const float cmd) override;

    bool isCommunicationError() override;

    // Arms the watchdog (see MotorDriverInterface) and then engages the motor - the velocity
    // controller does nothing until engaged, and a tripped failsafe disengages it.
    void armFailsafe() override;

    void resetFailsafe() override;

    bool isFailsafeTripped() override;

    MotorCommandStats getCommandStats() const override;

    std::uint8_t getFaultFlags() const override;

    // Pure logic, factored out so it's unit-testable without any Phidget SDK handles: whether a
    // PhidgetReturnCode from a command means the hardware watchdog (or the E-Stop input, which
    // the board reports the same way) rejected it.
    static bool isFailsafeTrippedReturnCode(const PhidgetReturnCode res);

    // What armFailsafe() has to do to leave the channel with a working, freshly-fed watchdog.
    // A Phidget channel's failsafe can only be enabled once per open, and once it has tripped the
    // channel rejects every further call until it is closed and re-opened (Phidget Failsafe
    // Guide) - so neither resetFailsafe() nor enableFailsafe() can recover a tripped channel on
    // their own.
    enum class FailsafeAction
    {
        kEnable,           // Never enabled on this open channel - enable it.
        kFeed,             // Already enabled and healthy - just reset (feed) the timer.
        kReopenAndEnable,  // Tripped - close/re-open the channel, reconfigure, then enable.
        kNone,             // The failsafe is configured off (motor_failsafe_enabled=false) - do nothing.
    };

    // Pure logic, factored out so it's unit-testable without any Phidget SDK handles.
    // `configured` is DrivetrainSettings::motor_failsafe_enabled; when false the watchdog is never
    // armed, so the other two inputs are irrelevant (nothing can trip).
    static FailsafeAction selectFailsafeAction(
        const bool configured, const bool enabled, const bool tripped);

    // How a velocity-controller error event changes this board's state. Pure logic, factored out
    // so it's unit-testable without any Phidget SDK handles.
    struct ErrorEventEffect
    {
        std::uint8_t set_fault_bits{0};
        // Fault bits the event clears (EEPHIDGET_OK, "Error Cleared", only clears the transient
        // ones - a latched condition needs resetFailsafe()'s re-open).
        std::uint8_t clear_fault_bits{0};
        // The board disengaged the motor / rejects commands until the channel is re-opened, so
        // the failsafe-tripped latch is set and recovery goes through resetFailsafe().
        bool latches_failsafe{false};
    };

    static ErrorEventEffect errorEventEffect(const Phidget_ErrorEventCode code);

    // Fault bits that clear by themselves when the board reports the condition is gone.
    static std::uint8_t transientFaultBits();

    // Motor-shaft revolutions, in encoder lines, covered at `rpm` for `dt_s` seconds: the
    // position fallback when the Encoder channel can't be opened. 0 for non-positive dt_s/lines.
    static double motorRpmToLines(const double rpm, const double dt_s, const float lines);

    // How long without a velocity event before the reported speed is forced to 0, for a
    // controller reporting every `data_interval_ms`. 3x the interval so one or two late/missed
    // events don't zero the speed of a turning wheel, but never below kMinVelocityStaleTimeout:
    // at a board-minimum interval of a few ms, 3x would be shorter than an ordinary SDK-thread
    // scheduling hiccup.
    static std::chrono::nanoseconds encoderStaleTimeout(const std::uint32_t data_interval_ms);

    static constexpr std::chrono::milliseconds kMinVelocityStaleTimeout{100};

    // The velocity target actually posted for a commanded motor RPM: non-finite commands become
    // 0, then clamped to +-max_rpm, then negated for a reversed motor. Pure logic, factored out so
    // it's unit-testable without any Phidget SDK handles.
    static float motorTargetRpm(const float cmd, const float max_rpm, const bool reversed);

private:

    // Applies the velocity controller's settings (rescale factor first - every other value is
    // in its units - then data interval, limits, gains, failsafe behaviour) to the open
    // motor_handle_, and logs what the board ended up with. Used by initialize() and again after
    // reopenMotorChannel(), since closing the channel drops them. Throws on an SDK error or on a
    // configured value outside the board's range.
    void configureMotorChannel();

    // Opens the Encoder channel for joint position; on failure logs a warning and leaves
    // encoder_handle_ null, so position is integrated from the velocity feedback instead.
    void openEncoderChannel(const std::uint32_t data_interval_ms);

    // Closes and re-opens motor_handle_ (the same handle - never deleted, so a concurrent
    // command_worker_ call on it just fails, it never dangles) and re-applies
    // configureMotorChannel(). The only way out of a tripped failsafe / E-Stop / stall.
    // Blocking - never call from the RT path.
    void reopenMotorChannel();

    // PhidgetMotorVelocityController_enableFailsafe() with the configured timeout; throws on
    // failure (including a timeout outside the board's getMin/MaxFailsafeTime()).
    void enableFailsafe();

    void setEngaged(const bool engaged);

    // armFailsafe() body once the latch has been taken: enable / feed / re-open the watchdog as
    // selectFailsafeAction() says, zero the target and engage. Throws on failure.
    void armAndEngage(const bool tripped);

    // Body of command_worker_: takes the newest target from command_mailbox_ and sends it.
    void commandWorkerLoop();

    void startCommandWorker();
    void stopCommandWorker();

    // "0x3b (Failsafe Triggered)"-style text for a PhidgetReturnCode, for exception messages.
    static std::string returnCodeToString(const PhidgetReturnCode ret);

    // Pure logic, factored out so it's unit-testable without any Phidget SDK handles.
    static bool isCommTimedOut(
        const std::chrono::steady_clock::time_point & last_update,
        const std::chrono::steady_clock::time_point & now,
        const std::chrono::nanoseconds & timeout);

    static void CCONV velocityChangeHandler(
        PhidgetMotorVelocityControllerHandle phid,
        void * ctx,
        double velocity);

    static void CCONV positionChangeHandler(
        PhidgetEncoderHandle phid,
        void * ctx,
        int positionChange,
        double timeChange,
        int indexTriggered);

    static void CCONV currentChangeHandler(
        PhidgetCurrentInputHandle phid,
        void * ctx,
        double current
    );

    static void CCONV temperatureChangeHandler(
        PhidgetTemperatureSensorHandle phid,
        void *ctx,
        double temperature
    );

    static void CCONV errorHandler(
        PhidgetHandle phid,
        void * ctx,
        Phidget_ErrorEventCode code,
        const char * description);

    // Sets motor_inductance on attach - the only point at which it skips the board's motor
    // characterization, which fails while the motor contactor is open.
    static void CCONV attachHandler(
        PhidgetHandle phid,
        void * ctx);

    std::weak_ptr<PhidgetDriver> driver_;

    const DrivetrainSettings settings_;

    const std::uint8_t channel_;
    std::int32_t serial_number_;

    PhidgetMotorVelocityControllerHandle motor_handle_{nullptr};
    PhidgetEncoderHandle encoder_handle_{nullptr};
    PhidgetCurrentInputHandle current_handle_{nullptr};
    PhidgetTemperatureSensorHandle temperature_handle_{nullptr};

    // `state_` is written from the Phidget SDK's callback thread (velocity/position/current/
    // temperature handlers) and read from the RT control thread via readState(). `state_mtx_`
    // guards `state_` itself; `state_snapshot_` is the last successfully-read copy, only ever
    // touched from readState() on the RT thread, so readState() never blocks the RT loop waiting
    // for the SDK thread to release the lock.
    std::mutex state_mtx_;
    MotorDriverState state_{};
    MotorDriverState state_snapshot_{};

    // Set once openEncoderChannel() has given up on the Encoder channel. Atomic rather than a
    // check of encoder_handle_: the velocity callback (SDK thread) already fires while
    // openEncoderChannel() runs on the activating thread.
    std::atomic<bool> position_from_velocity_{false};

    // Position fallback (position_from_velocity_): integrated motor revolutions in lines (SDK
    // callback thread only) and the time of the previous velocity event.
    double integrated_lines_{0.0};
    // Atomic only so reopenMotorChannel() can reset it (0 = no previous event) from its thread.
    std::atomic<std::int64_t> last_velocity_event_for_integration_ns_{0};

    // steady_clock ns of the last velocity event; written on the SDK thread, read in readState().
    std::atomic<std::int64_t> last_velocity_event_ns_{0};

    // encoderStaleTimeout() of the interval the controller actually runs at; set when configured.
    std::atomic<std::int64_t> velocity_stale_timeout_ns_{encoderStaleTimeout(50).count()};

    const bool direction_reversed_;

    // RT thread -> command_worker_ hand-off of the newest motor-RPM target (already sign-
    // corrected for direction_reversed_).
    LatestValueMailbox<float> command_mailbox_;
    std::thread command_worker_;
    // true while command_worker_ is inside the blocking setTargetVelocity call.
    std::atomic<bool> command_in_flight_{false};

    // Instrumentation: submitted = sends by command_worker_, dropped_pending = targets superseded
    // in the mailbox before being sent, latency = duration of the blocking call. Relaxed
    // atomics, no locks.
    CommandStatsRecorder command_stats_;

    // Set when a command is rejected with EPHIDGET_FAILSAFE or an error event says the board
    // disengaged (failsafe, E-Stop, stall, energy dump); read via isFailsafeTripped() (RT
    // thread). Only cleared by resetFailsafe() - an operator-driven action.
    std::atomic<bool> failsafe_tripped_{false};

    // FaultFlag bits raised by errorHandler(); see errorEventEffect().
    std::atomic<std::uint8_t> fault_bits_{0};

    // Whether enableFailsafe() has succeeded on the currently open motor channel. Cleared by
    // reopenMotorChannel() (closing the channel disables the failsafe). Only touched from
    // armFailsafe()/resetFailsafe(), serialized by failsafe_mtx_.
    bool failsafe_enabled_{false};

    // Serializes armFailsafe()/resetFailsafe() (on_activate() vs. the latch-reset service).
    std::mutex failsafe_mtx_;

    // Written from the Phidget SDK callback thread each time a telemetry callback fires - the
    // firing itself is the liveness signal, independent of the value carried. Read from the RT
    // thread via isCommunicationError().
    std::atomic<std::int64_t> last_update_time_ns_{0};

    const std::chrono::nanoseconds comm_timeout_;

    rclcpp::Logger logger_{rclcpp::get_logger("PhidgetMotorDriver")};
};

}  // namespace isaacbot_amr_hardware_interface

#endif  // ISAACBOT_AMR_HARDWARE_INTERFACE_ISAACBOT_AMR_DRIVER_PHIDGET_MOTOR_DRIVER_PHIDGET_MOTOR_DRIVER_HPP_
