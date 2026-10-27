# isaacbot_amr_hardware_interface

ros2_control hardware plugins for Ardennes. It exports two plugins and has no launch files or
config of its own:

- **`ArdennesSystem`** drives the four wheel motors and the safety controller.
- **`PhidgetImuSensor`** reads the IMU.

## Plugins

| Plugin (`<ros2_control>` `<plugin>`) | Base class | Hardware | Exports |
|--------------------------------------|------------|----------|---------|
| `isaacbot_amr_hardware_interface/ArdennesSystem` | `hardware_interface::SystemInterface` | 4× Phidget DCC1020 DC motor controllers (Phidget22 SDK, on-board speed loop), safety controller over Modbus TCP | per wheel joint: `velocity` command; `position`, `velocity`, `effort` state |
| `isaacbot_amr_hardware_interface/PhidgetImuSensor` | `hardware_interface::SensorInterface` | Phidgets Spatial MOT0110, Madgwick orientation filter | `imu/orientation.{x,y,z,w}`, `imu/angular_velocity.*`, `imu/linear_acceleration.*` |

The URDF that wires them lives in `isaacbot_amr_description`:
- `urdf/ardennes/ardennes_macro.urdf.xacro`: component `isaacbot_amr_system_node`, with the Modbus
  endpoint, timeouts and motor/gearbox/encoder parameters as `<param>` tags.
- `urdf/common/imu.urdf.xacro`: component `isaacbot_amr_imu`.

The IMU parameters are declared in `src/phidgets_spatial_parameters.yaml`. `isaacbot_amr_controller`
starts `controller_manager` and the controllers on top of these plugins.

Reference: DCC1020 product page <https://www.phidgets.com/?prodid=1333> (motor controller),
`docs/MOT0110_reference.pdf` (IMU). `docs/DCC1000_reference.pdf` is the previous motor controller,
kept for reference.

## Dependencies

The Phidget22 SDK (`libphidget22`, `libphidget22-dev` >= 1.26) comes from the official Phidgets apt
repo and is linked through pkg-config. It is not a rosdep key, so install it before `rosdep`:

```bash
curl -fsSL https://www.phidgets.com/downloads/setup_linux | sudo -E bash -
sudo apt install libphidget22 libphidget22-dev
```

`phidgets_api` / `ros-lyrical-libphidget22` are no longer used: they bundle 1.19, which has no
DCC1020 support. The IMU goes through the in-package `PhidgetSpatialDevice` wrapper over the C API.
The balenaOS image (`isaacbot_amr_balenaos/isaacbot_amr_platform/Dockerfile`) installs the SDK the
same way. Update each DCC1020 to firmware v104 or later with `phidget22admin` (fixes a
data-interval change-event bug).

## Motor controllers (DCC1020)

One DCC1020 per wheel on VINT hub ports 0 (FL), 1 (FR), 4 (RL), 5 (RR); FL and RL are reversed.
Each runs the wheel speed loop on board (`MotorVelocityController`: trapezoidal profile + PID).

- **Command.** `write()` converts each wheel command to a motor-shaft RPM target (rad/s ×
  `gear_ratio` × 60 / 2π), clamped to ±`max_rpm_motor_speed`. The controller is rescaled to motor
  RPM (`rescaleFactor = 60 / (4 · encoder_resolution)`). There is no asynchronous setter, so
  `sendCmdVel()` posts the newest target lock-free into a per-motor `LatestValueMailbox`, and a
  per-motor command worker makes the blocking `PhidgetMotorVelocityController_setTargetVelocity`
  call. Targets replaced before they were sent count as `dropped_pending` in the `command path`
  diagnostic. Every send feeds the board failsafe, so a stalled RT loop still trips it.
- **Feedback.** Velocity from the board's `OnVelocityChange` events (always the board's minimum
  data interval; the Encoder channel follows it); joint position from the Encoder channel, or integrated from the
  velocity if the Encoder channel cannot be opened next to the controller (to verify on hardware).
  Current is reported in 10 mA units (`raw_current_to_amps_scale` 0.01).
- **Activation.** `armFailsafe()` enables, feeds or re-opens the failsafe, then engages the
  controller (`setEngaged(true)`); the board does nothing until engaged, and a trip disengages it.
- **Faults.** Board error events set `DriverState.fault_flags` bits, shown as `fault_flag.*` in
  diagnostics: `emergency_stop` (DCC1020 E-Stop input, latched), `over_temperature`,
  `over_current`, `over_voltage` (also energy dump, latched), `motor_stall` (latched), `bad_power`.
  Transient bits clear on the board's "Error Cleared" event. Latched trips (failsafe, E-Stop, stall,
  energy dump) are cleared by `hardware_interface/sw_e_stop_latch_reset`, which re-opens the
  channel. The `FaultFlag` message still has only `emergency_stop` and `motor_setup_fault`.
- **Parameters.** On-board loop settings are optional URDF `<param>`s (`velocity_kp/ki/kd`,
  `velocity_deadband_rpm`, `motor_acceleration_rpm_s`, `motor_surge_current_limit`,
  `motor_stall_velocity_rpm`, `motor_failsafe_current_limit`, `motor_inductance`,
  `motor_failsafe_braking_enabled`); absent means board default, logged at configure. The DCC1000
  parameters `motor_acceleration` and `motor_supply_voltage`, and the removed
  `motor_data_interval_ms` override, are rejected by `on_init()`.

## ROS interface (`isaacbot_amr_hardware_controller` node)

`ArdennesSystem` creates a helper node, `isaacbot_amr_hardware_controller`, inside the
`controller_manager` process. It inherits that process's namespace and `diagnostics` remap.

| Direction | Name | Type |
|-----------|------|------|
| pub | `hardware_interface/driver_state` | `isaacbot_amr_msgs/IsaacbotAmrDriverState` |
| pub | `hardware_interface/safety_status` | `isaacbot_amr_msgs/SafetyStatus` - plant state: HW E-Stop button, contactor feedback, latch, link health |
| pub | `hardware_interface/safety_command_echo` | `isaacbot_amr_msgs/SafetyCommandEcho` - read-backs of the coils *we* drive; diagnostic, may only ever inhibit |
| pub | `hardware_interface/aux_io_state` | `isaacbot_amr_msgs/AuxIoState` - general-purpose aux IO (PLC DIO06..11 inputs, DIO00..05 output read-backs). Not safety |
| srv | `hardware_interface/sw_user_e_stop_set` | `std_srvs/Trigger` - set the software E-Stop |
| srv | `hardware_interface/sw_user_e_stop_reset` | `std_srvs/Trigger` - reset the software E-Stop |
| srv | `hardware_interface/sw_e_stop_latch_reset` | `std_srvs/Trigger` - clear the safety relay latch |
| srv | `hardware_interface/aux_output_<0..5>/set` | `std_srvs/SetBool` - switch aux output DIO00..05 ON/OFF; replies after the PLC acknowledges the write |
| pub | `diagnostics` | hardware id `Isaacbot System`: driver and safety controller status |

The aux IO topic and services exist only on real hardware: in simulation `gz_ros2_control`
replaces `ArdennesSystem`, so nothing talks to the PLC.

Consumers: `isaacbot_amr_safety` (driver state, the SW E-Stop echo, e-stop set), `isaacbot_amr_twist_mux`'s
`isaacbot_amr_motion_lock_node` (both safety topics), `isaacbot_amr_crsf_teleop` (safety status + e-stop
services) and the Foxglove dashboard.

The two safety topics replaced a single `gpio_state` of seven undifferentiated bools. Plant state
and command read-backs are separated because the difference decides what a consumer is allowed to
conclude — see `isaacbot_amr_arch/SAFETY_CHAIN.md` §4 and the message files themselves.

```bash
ros2 topic echo /isaacbot/hardware_interface/safety_status
ros2 service call /isaacbot/hardware_interface/sw_user_e_stop_reset std_srvs/srv/Trigger
ros2 control list_hardware_components
```

## Layout

```
domain/                    emergency stop, velocity command guard, error filter, IMU calibration,
                           driver/GPIO ports - no ROS, hardware_interface or vendor SDK
application/               IsaacbotControlLoopUseCase - the read()/write() cycle decisions
isaacbot_amr_driver/              Phidget DCC1020 motor driver adapters (Phidget22), LatestValueMailbox
isaacbot_amr_safety_controller/   safety controller E-Stop / GPIO adapter over Modbus
isaacbot_amr_sensors/             PhidgetImuSensor, PhidgetSpatialDevice (C API wrapper)
isaacbot_amr_system/              IsaacbotSystem base + ArdennesSystem (the ros2_control plugin)
system_ros_interface/      isaacbot_amr_hardware_controller node: topics, services, diagnostics
```

The Modbus TCP client itself lives in `isaacbot_amr_transport/isaacbot_amr_modbus_driver` - it was
extracted from this package so it could be depended on and tested on its own.

## Tests

```bash
colcon test --packages-select isaacbot_amr_hardware_interface && colcon test-result --all --verbose
```

- `test/` has gtests per layer: domain, application, driver, safety controller,
  sensors, system and the ROS interface. Fakes live in `test/fakes/`.
- Three architecture checks run as CTest tests:
  - `scripts/check_domain_purity.sh`: domain code includes no ROS, hardware_interface or
    vendor headers.
  - `scripts/check_application_purity.sh`: the same rule for application code.
  - `scripts/check_rt_path_purity.sh`: no known-blocking Modbus or Phidget SDK calls on the
    `read()`/`write()` real-time path, including the blocking `PhidgetMotorVelocityController_*`
    setters.
