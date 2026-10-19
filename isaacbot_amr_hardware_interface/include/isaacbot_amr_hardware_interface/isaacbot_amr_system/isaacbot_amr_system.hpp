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

#ifndef ISAACBOT_AMR_HARDWARE_INTERFACE_ISAACBOT_AMR_SYSTEM_ISAACBOT_AMR_SYSTEM_HPP_
#define ISAACBOT_AMR_HARDWARE_INTERFACE_ISAACBOT_AMR_SYSTEM_ISAACBOT_AMR_SYSTEM_HPP_

#include <array>
#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <diagnostic_updater/diagnostic_status_wrapper.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <hardware_interface/handle.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include "hardware_interface/hardware_component_interface.hpp"

#include "isaacbot_amr_hardware_interface/domain/isaacbot_amr_driver.hpp"
#include "isaacbot_amr_hardware_interface/system_ros_interface/system_ros_interface.hpp"

#include "isaacbot_amr_hardware_interface/application/isaacbot_amr_control_loop_use_case.hpp"
#include "isaacbot_amr_hardware_interface/domain/command_path_health.hpp"
#include "isaacbot_amr_hardware_interface/domain/emergency_stop.hpp"
#include "isaacbot_amr_hardware_interface/domain/isaacbot_amr_error_filter.hpp"
#include "isaacbot_amr_hardware_interface/domain/isaacbot_amr_gpio_port.hpp"
#include "isaacbot_amr_hardware_interface/domain/velocity_command_guard.hpp"

namespace isaacbot_amr_hardware_interface
{

using return_type = hardware_interface::return_type;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using StateInterface = hardware_interface::StateInterface;
using CommandInterface = hardware_interface::CommandInterface;

// IsaacbotSystem is the generic hardware_interface::SystemInterface adapter: lifecycle, joint
// bookkeeping, the RT read()/write() cycle, and the domain-port-only view of the driver/safety
// controller (IsaacbotDriverInterface, IsaacbotGpioPort, EmergencyStopInterface). It has no knowledge of
// any concrete isaacbot variant's backend. A concrete isaacbot (e.g. ArdennesSystem in ardennes_system.hpp,
// the Phidget+Modbus variant) derives from this class and implements only the pure-virtual
// extension points below (defineIsaacbotDriver(), defineIsaacbotController(), updateHwStates(),
// getSpeedCmd(), ...). If you're adding a new use case or domain rule, it belongs here or in
// domain/application/ - not in a per-variant subclass; if you're adding isaacbot-specific wiring
// (a new backend, new URDF params), it belongs in the subclass instead.
//
// Real-time contract for read()/write() (see .claude/rules/ros2_control_architecture.md §5):
//   - Driver feedback is a cached snapshot refreshed opportunistically under an uncontended lock
//     (IsaacbotDriverInterface::getData(), backed by PhidgetIsaacbotDriver) - never blocks on the
//     diagnostics thread.
//   - GPIO/E-Stop IO reads (IsaacbotGpioPort::queryControlInterfaceIOStates(),
//     EmergencyStopInterface::readEStopState()/readEStopLatchState()) use try_lock and return the
//     last-known-good value on contention rather than blocking, since the actual Modbus polling
//     happens on a dedicated background thread (see ContactCoilHandler).
//   - All read()-path publishing goes through realtime_tools::RealtimePublisher (see
//     SystemROSInterface), never a plain rclcpp::Publisher::publish() call.
//   - Motor commands in write() use the Phidget SDK's async call
//     (PhidgetDCMotor_setTargetVelocity_async()) and drop the command rather than block if a
//     prior async call hasn't completed yet.
//   - handleIsaacbotDriverWriteOperation() delegates the actual write attempt to
//     IsaacbotControlLoopUseCase::performWriteOperation(), which also uses try_lock and skips the
//     write cycle on contention instead of blocking (see application/isaacbot_amr_control_loop_use_case.hpp).
//   - Accepted exception: RCLCPP_*_STREAM_THROTTLE calls on error/contention branches build a
//     std::stringstream (allocates) but are throttled to at most once per 5s and only fire off
//     the happy path - a deliberate tradeoff, not an oversight.
class IsaacbotSystem : public hardware_interface::SystemInterface
{

public:

    explicit IsaacbotSystem(const std::vector<std::string> & joint_order);

    virtual ~IsaacbotSystem() = default;

    CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams & params) override;
    CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;
    CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

    std::vector<StateInterface> export_state_interfaces() override;
    std::vector<CommandInterface> export_command_interfaces() override;

    return_type read(const rclcpp::Time & time, const rclcpp::Duration & /* period */) override;
    return_type write(const rclcpp::Time & /* time */, const rclcpp::Duration & /* period */) override;

protected:

    void checkJointSize() const;
    void sortAndCheckJointNames();
    void setInitialValues();
    void checkInterfaces() const;

    void readDrivetrainSettings();
    void readDriverStatesUpdateFrequency();
    void readDriverInitAndActivationAttempts();
    void readErrorFilterMaxErrorsCounts();
    void readEStopSettings();

    void configureIsaacbotController();
    void configureIsaacbotDriver();

    // Shared by on_cleanup()/on_shutdown(): both tear down the same set of owned components, and
    // on_shutdown() in particular must tolerate being called from Unconfigured (isaacbot_amr_driver_ /
    // isaacbot_amr_controller_ / e_stop_ / system_ros_interface_ still null - on_configure() was never
    // reached), which the null check on isaacbot_amr_driver_ guards against.
    void teardownIsaacbotComponents();

    virtual void defineIsaacbotDriver() = 0;

    // Reads whatever URDF hardware_parameters this isaacbot variant's safety-controller backend
    // needs (e.g. Modbus host/port for ArdennesSystem) into a member owned by the subclass. This
    // is a pure virtual extension point rather than a concrete IsaacbotSystem method (unlike
    // readDrivetrainSettings() etc. above) because, unlike the drivetrain parameters, the
    // safety-controller transport is backend-specific: IsaacbotSystem itself has no business
    // requiring a "modbus_host" parameter to exist. Called from on_init() inside the same
    // try/catch as the other read*Settings() calls, so any std::exception it throws (e.g.
    // std::invalid_argument for a missing/empty parameter) is handled uniformly. See
    // defineIsaacbotController() for the matching construction-time extension point.
    virtual void readIsaacbotControllerSettings() = 0;

    // Pure virtual extension point (mirrors defineIsaacbotDriver()): constructs and stores
    // isaacbot_amr_controller_ (IsaacbotGpioPort) and e_stop_ (EmergencyStopInterface) using whatever
    // safety-IO backend this isaacbot variant uses. IsaacbotSystem only depends on these two domain
    // ports here - never on a concrete backend type - so a isaacbot variant using a different
    // safety-IO transport (e.g. CAN or direct GPIO instead of Modbus) only has to provide a new
    // implementation of this method, not modify IsaacbotSystem. configureIsaacbotController() (which
    // calls this) then starts the safety IO through IsaacbotGpioPort::start() and releases the
    // start-up E-Stop triggers through EmergencyStopInterface::releaseStartupTriggers(). So
    // e_stop_'s IO port must drive the same safety-IO instance isaacbot_amr_controller_ starts;
    // otherwise the release silently no-ops. See ArdennesSystem::defineIsaacbotController() for the
    // reference (Modbus-backed) implementation, which shares one IsaacbotSafetyController.
    virtual void defineIsaacbotController() = 0;

    void resetEStop();
    void resetEStopLatch();

    void updateMotorsState(const rclcpp::Time & time);
    void updateDriverState();
    void updateCommunicationStatus();
    void updateMotorsStateDataTimedOut();
    void updateDriverStateDataTimedOut();
    void updateFlagErrors();
    void updateEStopState();

    void updateContactorPlausibility();

    double readPositiveToleranceParam(
        const std::string & key, const double default_value) const;

    void diagnoseSafetyLink(diagnostic_updater::DiagnosticStatusWrapper & status);

    // Command-path instrumentation: per-wheel issued/dropped/completed counts and latencies plus
    // what each write() cycle did. Runs on the diagnostics thread, reads lock-free counters only.
    void diagnoseCommandPath(diagnostic_updater::DiagnosticStatusWrapper & status);

    // Is imu/data valid and fresh? Judges the data, so it catches an IMU component that never
    // activated (the broadcaster then publishes NaN), a detached device and a stopped broadcaster.
    void diagnoseImuData(diagnostic_updater::DiagnosticStatusWrapper & status);
    virtual void updateHwStates(const rclcpp::Time & time) = 0;

    virtual void updateDriverStateMsg() = 0;

    void handleIsaacbotDriverWriteOperation(std::function<void()> write_operation);

    // Logs the outcome of a IsaacbotControlLoopUseCase::performWriteOperation() call at the same
    // throttled WARN level and with the same messages this package used before the write-path
    // decision logic moved into IsaacbotControlLoopUseCase. A no-op on kSucceeded.
    void logWriteOperationResult(const WriteOperationResult & result);

    // The E-Stop reset invariant, as seen by the service-callback thread. Returns the flag
    // published by refreshVelocityCommandsZeroFlag() on the RT thread rather than inspecting
    // hw_commands_velocities_ directly: the executor thread must not read that non-atomic
    // std::vector<double> while controller_manager's RT thread is writing it.
    bool areVelocityCommandsNearZero();

    // Recomputes commands_are_zero_ from the current hw_commands_velocities_. Called at the top
    // of write() (i.e. on the RT thread, every cycle, whether or not motion is being commanded)
    // so the flag the service thread reads is never more than one control period stale.
    void refreshVelocityCommandsZeroFlag();

    // Zeroes hw_commands_velocities_ and speed_cmd_buffer_. Both are pre-sized in
    // setInitialValues(), so this allocates nothing and is safe to call from write().
    void zeroVelocityCommands();

    // Fills the (already correctly-sized) `speed_cmd` buffer in place — no allocation, so it's
    // safe to call from write() every RT cycle.
    virtual void getSpeedCmd(std::vector<float> & speed_cmd) const = 0;

    virtual void diagnoseErrors(diagnostic_updater::DiagnosticStatusWrapper & status) = 0;
    virtual void diagnoseStatus(diagnostic_updater::DiagnosticStatusWrapper & status) = 0;

    const size_t joint_size_;

    const std::vector<std::string> joint_order_;
    std::vector<std::string> joints_names_sorted_;

    std::vector<double> hw_commands_velocities_;
    std::vector<double> hw_states_positions_;
    std::vector<double> hw_states_velocities_;
    std::vector<double> hw_states_efforts_;

    // Reused across write() cycles by getSpeedCmd() to avoid allocating on the RT thread.
    // Sized once in setInitialValues().
    std::vector<float> speed_cmd_buffer_;

    // Isaacbot driver interface
    std::shared_ptr<IsaacbotDriverInterface> isaacbot_amr_driver_;
    // Isaacbot safety controller GPIO port: start, read()-path GPIO polling, link health and aux
    // outputs go through this domain port, never a concrete backend type; E-Stop coil writes go
    // only through e_stop_. defineIsaacbotController() (implemented per isaacbot variant, e.g.
    // ArdennesSystem's Modbus-backed one) constructs the concrete adapter as a scoped local and
    // never stores it as a member, so nothing on the RT read()/write() path can reach it directly.
    std::shared_ptr<IsaacbotGpioPort> isaacbot_amr_controller_;
    // Isaacbot emergency stop interface
    std::shared_ptr<EmergencyStopInterface> e_stop_;
    // Debounced/latched per-category error state (write cmds, read-motor-states,
    // read-driver-state, fault-flag). Diagnostics-only: never gates read()/write() return codes
    // or triggers a lifecycle transition. Built once in on_init() (readErrorFilterMaxErrorsCounts())
    // from URDF thresholds; unlike isaacbot_amr_driver_/isaacbot_amr_controller_/e_stop_ it owns no hardware
    // handles, so it is NOT reset in on_cleanup()/on_shutdown()/on_error().
    std::unique_ptr<IsaacbotErrorFilter> isaacbot_amr_error_filter_;
    // Owns the RT-loop decision logic (E-Stop state, fault-flag reset, write-command gating and
    // serialization) that previously lived directly in IsaacbotSystem's own methods - see
    // application/isaacbot_amr_control_loop_use_case.hpp. Constructed once isaacbot_amr_driver_/e_stop_ are
    // both ready (end of on_configure()) and reset alongside them in teardownIsaacbotComponents(),
    // so it always references the same live isaacbot_amr_driver_/e_stop_/isaacbot_amr_error_filter_ this
    // IsaacbotSystem instance owns.
    std::unique_ptr<IsaacbotControlLoopUseCase> control_loop_use_case_;
    // Cached result of the last updateEStopState() poll; gates write() while true.
    // Defaults to true (fail-safe: no motion commands until confirmed clear).
    std::atomic_bool e_stop_active_{true};
    // Largest |command| seen by the last refreshVelocityCommandsZeroFlag() pass, published so
    // the refusal can name a number instead of just saying "not zero". Diagnostics only.
    std::atomic<double> max_abs_velocity_command_{0.0};
    std::atomic<double> max_abs_velocity_state_{0.0};
    // Cached result of the last refreshVelocityCommandsZeroFlag() pass; gates the E-Stop reset.
    // Defaults to false (fail-safe: refuse the reset until an RT cycle has actually observed a
    // zero command). resetEStop() already requires PRIMARY_STATE_ACTIVE, where write() ticks at
    // the controller_manager's update_rate, so this is populated well before any reset can land.
    std::atomic_bool commands_are_zero_{false};

    // Second half of the E-Stop reset invariant: the wheels must also not actually be
    // turning. Fail-safe default false, like commands_are_zero_.
    std::atomic_bool states_are_zero_{false};

    // Drive train system settings
    DrivetrainSettings drivetrain_settings_;

    // ROS hardware interface
    std::unique_ptr<SystemROSInterface> system_ros_interface_;

    // Imported from URDF
    unsigned max_isaacbot_amr_driver_initialization_attempts_;
    unsigned max_isaacbot_amr_driver_activation_attempts_;
    // Deadband (rad/s at the wheel) under which a velocity command counts as zero for the E-Stop
    // reset invariant. Optional in the URDF, unlike the parameters above - absent means
    // kDefaultVelocityCommandZeroTolerance, so an existing URDF keeps working unchanged.
    double velocity_command_zero_tolerance_{kDefaultVelocityCommandZeroTolerance};
    double velocity_state_zero_tolerance_{kDefaultVelocityStateZeroTolerance};

    rclcpp::Logger logger_{rclcpp::get_logger("IsaacbotSystem")};
    rclcpp::Clock steady_clock_{RCL_STEADY_TIME};

    // Lazily seeded from the first `time` argument read() receives (see
    // IsaacbotSystem::read()) rather than default-constructed with a fixed clock type: the
    // controller_manager passes `time` using its own node clock (RCL_ROS_TIME by default),
    // and rclcpp::Time comparisons throw std::runtime_error when the two operands use
    // different clock types. Seeding from the actual `time` guarantees the clock types
    // always match, regardless of what clock the caller happens to use.
    // Diagnostics-thread state for the command-path and IMU tasks (one thread owns it).
    std::array<ErrorTrend, 4> command_error_trend_;
    std::uint64_t imu_prev_total_{0};
    std::int64_t imu_prev_ns_{0};

    bool driver_state_update_time_initialized_{false};
    rclcpp::Time next_driver_state_update_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Duration driver_states_update_period_{0, 0};
};

}  // namespace isaacbot_amr_hardware_interface

#endif  // ISAACBOT_AMR_HARDWARE_INTERFACE_ISAACBOT_AMR_SYSTEM_ISAACBOT_AMR_SYSTEM_HPP_
