# Open items

This page lists what the manual can't state yet, and the places where the repository disagrees
with itself. It was compiled on 2026-10-01 from `master` and updated on 2026-10-02 for the wheel PID merge and the stale-documentation fixes, and for the DCC1020 motor controller swap. When an item is resolved, update the
source file first, then the manual page, then remove the item here.

## Values not yet defined (TBD)

### Physical and environmental

| Item | Where it is needed | Requirement |
|---|---|---|
| Rated payload | [Specification](hardware/specification.md) | SYS-SR-027 |
| Maximum slope and terrain types | Specification | SYS-SR-028 |
| Operating temperature range | Specification | SYS-SR-029 |
| Ingress protection (IP) rating | Specification | SYS-SR-029 |
| Dimension limit (L × W × H) | Specification | SYS-SR-026 |
| Measured mass (scale) | Specification; the 58.67 kg is CAD | SYS-SR-001 |
| Measured stopping distance from 1.0 m/s | [Safety](safety.md) | SYS-SR-014 |
| Photos of the real isaacbot | [Overview](index.md) gallery | |
| Location of the hardware E-Stop and reset buttons on the chassis | Safety | SYS-SR-006 |
| Transport and handling: lifting points, towing, free-wheeling, storage | Safety | |

### Power

| Item | Where it is needed |
|---|---|
| Nominal pack voltage, cell count, energy (Wh) | [Specification](hardware/specification.md), [Power and battery](hardware/power-and-battery.md) |
| Runtime on hardware (SYS-SR-015) | Specification |
| Charger, charging connector, voltage/current, time and procedure (SYS-SR-018) | Power and battery |
| Main power switch: part and location; power-down order | Power and battery |
| How the LED controllers, router and IMU are powered | Power and battery |
| BMS telemetry rate (set by the ESP32 firmware, outside this repo) | Power and battery |

### DCC1020 motor controller swap

| Item | Where it is needed |
|---|---|
| Wire each DCC1020 hardware E-Stop input into the safety chain. It is always enabled and still jumpered as shipped | [Safety](safety.md), [Components](hardware/components.md) |
| Tune the on-board speed loop (`velocity_kp/ki/kd`, `velocity_deadband_rpm`, `motor_acceleration_rpm_s`, surge/stall/failsafe current, `motor_data_interval_ms`) and record the source of each value; the boards run on their defaults until then | [Drive and control](software/drive-and-control.md#wheel-speed-loop), `ardennes_macro.urdf.xacro` |
| Re-measure the `diff_drive` / Nav 2 acceleration limits (1.4 m/s², 1.3 rad/s², measured with the DCC1000) | Drive and control, `wheel_01_controller.yaml` |
| Bench-check that the Encoder channel opens next to the velocity controller (otherwise joint position is integrated from velocity) | Drive and control |
| Measure the DCC1020 command latency (`command path` diagnostic) before raising the controller rate above 25 Hz | Drive and control |
| Update each DCC1020 to firmware v104 or later with `phidget22admin` (fixes a data-interval change-event bug) | Components |
| Re-check `motor_current_limit` (15 A, ASSUMPTION) on the bench | Components |
| Fit a Power Guard (Phidgets recommends SAF2000 or SAF2020); part number **TBD** | [Power and battery](hardware/power-and-battery.md) |
| The CAD / URDF base mass still models the DCC1000 (78.8 g each), not the DCC1020 | [Specification](hardware/specification.md) |
| `isaacbot_amr_msgs/FaultFlag` carries only `emergency_stop` and `motor_setup_fault`; the new over-temperature, over-current, over-voltage, stall and bad-power bits are in diagnostics only | [ROS 2 API](software/ros-api.md) |
| Regenerate `rover_platform_mbse/export/rover_platform_export.sysml` with the MATLAB `build_all` | MBSE |

### Components and electrical

| Item | Where it is needed |
|---|---|
| Controller computer model, CPU and OS (the diagram only says "RPi") | [Components](hardware/components.md), [Software overview](software/overview.md) |
| Part numbers: motors (and power rating), BMS, battery pack, VINT hub, ELRS receiver, LED controllers, contactor/relays, E-Stop button, power board, Power Guard | Components |
| Portenta DIO electrical ratings and connector pinout | [IO and network](hardware/io-and-network.md) |
| USB vendor/product IDs and udev rules (the repo has none) | IO and network |
| Datasheets for the Portenta, motors, ELRS receiver, VINT hub and router | Components |
| Bind address of the Foxglove and rosbridge bridges | IO and network |

## Source conflicts

The manual uses the value the code runs with. The other source should be corrected.

| Topic | Value used by the code | Other value | Files |
|---|---|---|---|
| Track width | 0.617 m (CAD wheel joints; the tyre contact patches are 0.6136 m apart) | 0.615 m "measured" | `isaacbot_amr_description/config/wheel_01.yaml`, `isaacbot_amr_controller/config/wheel_01_controller.yaml` (comment) |
| Effective track (track × multiplier) | 0.617 × 1.659 = 1.0236 m | 1.020 m (controller comment); 1.0204 m (`effective_track_width` in `isaacbot_amr_crsf_teleop/config/isaacbot_amr_crsf_teleop.yaml`) | `isaacbot_amr_controller/config/wheel_01_controller.yaml` |
| `motor_current_limit` | 15.0 A (ASSUMPTION carried over from the DCC1000) | ~11.5 A (Phidgets: ~1.1× rated current, URDF comment); 10 A compiled-in default (`utils.hpp`) | `isaacbot_amr_description/urdf/ardennes/ardennes_macro.urdf.xacro`, `isaacbot_amr_hardware_interface/include/isaacbot_amr_hardware_interface/utils.hpp` |
| Maximum angular speed | 1.5 rad/s | "1.7 m/s2" (SYS-SR-004 answer; also a unit error) | `wheel_01_controller.yaml`, `rover_platform_mbse/system/*.xlsx` |
| Battery voltage | 24 V (system voltage; `motor_supply_voltage` was removed with the DCC1000) | 41.4 V in the simulated battery | `ardennes_macro.urdf.xacro`, `isaacbot_amr_description/urdf/common/battery.urdf.xacro` |
| BMS link | BLE → ESP32 → UDP | RS485/RS232 serial link (hardware part of the architecture diagram) | `isaacbot_amr_battery/`, `isaacbot_amr_arch/ardennes_arch.drawio` |
| Motor power switching | One contactor with an auxiliary contact | Four relays K1–K4 on QX 0.0, no auxiliary contact (schematic) | `isaacbot_amr_arch/SAFETY_CHAIN.md`, `isaacbot_amr_arch/ardennes_arch.drawio` |
| IMU hub port | -1 (any) is used when opening the device | 0 in the URDF, default 2 in the parameters | `isaacbot_amr_hardware_interface/src/phidget_imu_sensor.cpp` |
| SYS-SR-001 mass limit | — | The requirement text says ≤ 60 kg *excluding* payload; the Open Questions sheet answers that payload *is* included | `rover_platform_mbse/system/ARDENNES-ROVER_ROS_SYS-SR-V-002.xlsx` |
| MBSE baseline | Current values above | 100 Hz control and other non-CAD values (the CAD geometry and masses were re-read 2026-10-07) | `rover_platform_mbse/data/platform_parameters.json` (baseline 35aeb3e) |

## Behaviour worth reviewing

These come from reading the code while writing the manual. They are not documentation errors.

- `velocity_command_zero_tolerance` is still 0.4 rad/s. The reason for it (a frozen host PID
  integral) is gone since the host wheel PIDs are no longer spawned, so it can come down toward
  0.01 once that is verified on the isaacbot.
- The `E_STOP` LED animation follows only the hardware button. A software E-Stop, an RC E-Stop
  or a held latch leaves the LEDs on `READY`.
- The software motor-driver-fault input (`COIL_3`) is only asserted during start-up; nothing
  asserts it at runtime.
- The Foxglove layout (`isaacbot_amr_foxglove/`) uses topics the bridge whitelist in
  `isaacbot_amr_bringup/launch/isaacbot_amr_web_bridges.launch.py` leaves out, among them its joystick topic
  `teleop_foxglove_cmd_vel_stamped`. The whitelist is deliberate: the web joystick goes through
  `teleop_web_cmd_vel_stamped` and `isaacbot_amr_drive_mode`. The layout should follow, or those panels
  stay empty unless `ISAACBOT_AMR_PLATFORM_FOXGLOVE_TOPIC_WHITELIST` is opened.
