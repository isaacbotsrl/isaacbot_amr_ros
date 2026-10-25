# Safety

This chapter covers how the Ardennes stops, who and what can stop it, how to reset it, and what the LED panels tell you. Read it before you power the isaacbot for the first time.

!!! danger "Before first use"
    - **Find the hardware E-Stop button and the hardware reset button before you drive.** Their positions on the chassis are **TBD** in this manual.
    - **The motor contactor is the only hardware stop.** Everything in software (motion lock, zero commands, lidar guards) is defense in depth behind it.
    - **The isaacbot starts E-stopped.** Every start of the hardware interface leaves the safety latch set. You must reset the latch before the isaacbot moves (see [Reset after start-up](#reset-after-start-up)).
    - **The RC transmitter has the highest command priority.** The person with line of sight holding the transmitter has the last word. The RC transmitter, the Foxglove joystick and MANUAL mode bypass the lidar obstacle guards by design.
    - **The front and rear LEDs show the `E_STOP` animation only for the hardware button.** A software E-Stop or a held latch does *not* change the LEDs (see [LED status](#led-status)).
    - **The stop distance has not been measured.** It is **TBD**. The DCC1020's on-board motion profile (`motor_acceleration_rpm_s`, **TBD**, board default until tuned) also applies when slowing down, so it shapes every software stop.
    - **Lift the wheels off the ground for first tests and tuning runs.** The calibration tools only move the isaacbot when you pass `-p enable_motion:=true`.

ROS names on this page are relative to the robot namespace `isaacbot`. For example, `hardware_interface/sw_user_e_stop_set` is `/isaacbot/hardware_interface/sw_user_e_stop_set`.

## The E-Stop chain

Four sources can trip the safety relay (SET). Two can release it (RESET). The relay is a set-dominant latch inside the safety PLC. Its output drives the coil of the motor contactor. An auxiliary contact on the contactor reports back to the PLC whether the contacts are really closed.

```mermaid
flowchart LR
  HWB["HW E-Stop button"] --> SET
  SWU["SW user E-Stop<br/>(COIL_2)"] --> SET
  SWF["SW motor-driver fault<br/>(COIL_3)"] --> SET
  WDG["CPU watchdog timeout<br/>(COIL_1 heartbeat stops)"] --> SET
  HWR["HW reset button"] --> RST
  SWR["SW latch reset pulse<br/>(COIL_4)"] --> RST
  SET["SET (priority)"] --> L["SR latch in safety PLC"]
  RST["RESET"] --> L
  L --> K["Motor contactor"]
  K -->|"aux contact (COIL_0)"| FB["motor_contactor_engaged"]
  L -->|"COIL_5"| LS["latch status"]
  K --> M["4x DCC1020 motor drivers"]
```

Two properties matter:

- **Set-dominant.** A reset has no effect while any trip source is still asserted. The trip wins.
- **Latching.** The stop holds after the trip source clears. Only a deliberate reset releases it. A momentary fault cannot clear itself.

Source: `isaacbot_amr_arch/SAFETY_CHAIN.md` §1.

### Latch states

```mermaid
stateDiagram-v2
  [*] --> Latched : hardware interface starts (start-up asserts both SW inputs, then releases them)
  Clear --> Tripped : any SET source asserted
  Latched --> Tripped : any SET source asserted
  Tripped --> Tripped : reset ignored (set-dominant)
  Tripped --> Latched : all SET sources released
  Latched --> Clear : reset (HW reset button or sw_e_stop_latch_reset)
  Clear : contactor closed, motors powered
  Tripped : contactor open, trip source still active
  Latched : contactor open, waiting for reset
```

On start-up the safety controller writes the default state of each writable coil. The two software E-Stop inputs default to asserted. The hardware interface then releases them during configure, but the latch they set stays set until a reset.

Source: `isaacbot_amr_hardware_interface/src/isaacbot_amr_safety_controller/isaacbot_amr_safety_controller.cpp` (coil table, `initCoils()`), `isaacbot_amr_hardware_interface/include/isaacbot_amr_hardware_interface/domain/emergency_stop.hpp` (`releaseStartupTriggers()`).

### Safety PLC signals

The hardware interface talks to the safety PLC over Modbus TCP at `192.168.88.11:502`. For the link details see [IO and network](hardware/io-and-network.md).

| Signal | Modbus object | Direction | Meaning |
|---|---|---|---|
| `hw_e_stop_user_button` | `CONTACT_0` | PLC to ROS | Hardware E-Stop button pressed |
| `motor_contactor_engaged` | `COIL_0` | PLC to ROS | Contactor aux contact: contacts closed |
| `cpu_wdg_heartbeat` | `COIL_1` | ROS to PLC | Watchdog heartbeat (toggled) |
| `sw_e_stop_user_button` | `COIL_2` | ROS to PLC | Software user E-Stop (SET input) |
| `sw_e_stop_motor_driver_fault` | `COIL_3` | ROS to PLC | Software motor-driver fault (SET input) |
| `sw_e_stop_latch_reset` | `COIL_4` | ROS to PLC (pulse) | Latch reset |
| `sw_e_stop_latch_status` | `COIL_5` | PLC to ROS | Latch active |

Source: `isaacbot_amr_arch/SAFETY_CHAIN.md` §2, `isaacbot_amr_description/urdf/ardennes/ardennes_macro.urdf.xacro` (`modbus_host`, `modbus_port`).

!!! note
    In the current code the software motor-driver fault input (`COIL_3`) is asserted only during start-up. No runtime path asserts it. Motor-driver faults are handled in software instead (see [Motion gates](#motion-gates)).

### Watchdog heartbeat

The hardware interface toggles `COIL_1` on its own thread. If the toggling stops for longer than the relay's watchdog window, the relay trips and latches. This covers a crashed or hung computer.

| Parameter | Value | Note |
|---|---|---|
| `safety_wdg_kick_period_ms` | 200 ms | Heartbeat toggle period (5 Hz) |
| Relay watchdog window | ~1 s | Measured on the A1, a hardware property |
| `modbus_response_timeout_ms` | 150 ms | Bounds how long one slow transaction can delay a heartbeat |
| `safety_io_poll_period_ms` | 100 ms | IO poll (10 Hz), separate thread |

Source: `isaacbot_amr_description/urdf/ardennes/ardennes_macro.urdf.xacro`, `isaacbot_amr_arch/SAFETY_CHAIN.md` §3.

!!! warning
    Do not raise `safety_wdg_kick_period_ms` without re-measuring the relay's watchdog window. 200 ms gives about 5x margin against the ~1 s window.

### Software E-Stops

The hardware interface node `isaacbot_amr_hardware_controller` offers three `std_srvs/Trigger` services:

| Service | Effect | Refused when |
|---|---|---|
| `hardware_interface/sw_user_e_stop_set` | Asserts the SW user E-Stop (`COIL_2`), which trips the latch | Modbus write fails |
| `hardware_interface/sw_user_e_stop_reset` | Releases the SW user E-Stop | Hardware interface not ACTIVE; any wheel velocity command above `velocity_command_zero_tolerance` (0.4 rad/s); any measured wheel velocity above `velocity_state_zero_tolerance` (0.05 rad/s) |
| `hardware_interface/sw_e_stop_latch_reset` | Pulses `COIL_4` for `safety_latch_reset_pulse_ms` (100 ms). Also clears latched motor-failsafe, DCC1020 E-Stop / stall / energy-dump / motor-setup (open circuit at init, e.g. the contactor was open when the channel opened) and welded-contactor faults: it re-opens each motor channel, re-arms its watchdog and engages it again | Hardware interface not ACTIVE |

Source: `isaacbot_amr_hardware_interface/src/isaacbot_amr_system/isaacbot_amr_system.cpp` (`resetEStop()`, `resetEStopLatch()`, `areVelocityCommandsNearZero()`), `isaacbot_amr_description/urdf/ardennes/ardennes_macro.urdf.xacro`.

The state is published on two topics at 20 Hz (reliable, volatile, keep last 1):

| Topic | Type | Use |
|---|---|---|
| `hardware_interface/safety_status` | `isaacbot_amr_msgs/SafetyStatus` | Plant state: HW button, contactor, latch, `latch_cause`, `link_healthy`. May permit or inhibit motion. |
| `hardware_interface/safety_command_echo` | `isaacbot_amr_msgs/SafetyCommandEcho` | Read-backs of the coils ROS drives. May only ever inhibit motion. |

Source: `isaacbot_amr_arch/SAFETY_CHAIN.md` §4, `isaacbot_amr_msgs/msg/SafetyStatus.msg`, `isaacbot_amr_msgs/msg/SafetyCommandEcho.msg`, `driver_states_update_frequency` in `isaacbot_amr_description/urdf/ardennes/ardennes_macro.urdf.xacro`.

### RC switches and RC link failsafe

The ELRS RC teleop node (`isaacbot_amr_crsf_teleop_node`) calls the same services from two transmitter switches. Only a change of switch position fires a call.

| Channel | Switch position | Action |
|---|---|---|
| 5 (`e_stop_channel`) | low | `sw_user_e_stop_set` |
| 5 | high | `sw_user_e_stop_reset` |
| 4 (`e_stop_latch_reset_channel`) | low | `sw_e_stop_latch_reset` |

"Low" means a raw channel value below `channel_switch_threshold` (500). At start-up the node spends `switch_settle_frames` (100 ticks of 20 ms, 2 s) learning the switches' resting positions and fires nothing. Switches are ignored while the RC link is lost.

**RC link failsafe.** The link counts as lost when no frame arrives within `channel_timeout_ms` (200 ms), when link statistics are older than `link_stats_timeout_ms` (1000 ms), or when uplink link quality drops below `link_quality_lost_below` (30 %, recovers at 50 %). The node then publishes zeros for `zero_burst_duration_ms` (300 ms) and goes silent. The isaacbot stops and `twist_mux` falls through to its next input. **A lost RC link does not trip the E-Stop.**

The receiver's CRSF reaches the controller through the RUTX11 router over UDP, so a router reboot, a Serial Utilities reload or a pulled controller–router cable is handled the same way: a lost RC link, not an E-Stop. CRSF carries no authentication; `isaacbot_amr_crsf_udp_receiver` therefore accepts datagrams from the router's address (`192.168.1.1`) only, and the router drops AP clients' datagrams to the RC port. Neither stops a LAN host that forges the router's address.

Source: `isaacbot_amr_crsf_teleop/config/isaacbot_amr_crsf_teleop.yaml`, `isaacbot_amr_crsf_teleop/README.md`. More in [Teleop and LEDs](software/teleop-and-leds.md).

### Battery-driven E-Stop and shutdown

`isaacbot_amr_safety_node` (package `isaacbot_amr_safety`) watches `battery/battery_status` and acts through a behavior tree at 10 Hz:

| Battery reading | Action |
|---|---|
| health `WATCHDOG_TIMER_EXPIRE`, `DEAD` or `OVERVOLTAGE` | Calls `hardware_interface/sw_user_e_stop_set` |
| health `OVERHEAT` and temperature above `battery.temp.critical` (50 °C) | Calls `hardware_interface/sw_user_e_stop_set` |
| health `OVERHEAT` and temperature above `battery.temp.fatal` (60 °C) | Shuts down the ROS controller computer |
| anything else | Nothing |

The shutdown sequence trips the E-Stop, asks the hosts in `config/shutdown_hosts.yaml` to power off (the shipped file lists none), then runs `shutdown_ros_controller.sh`. You can also start it by hand:

```bash
ros2 service call /isaacbot/isaacbot_amr_safety_node/shutdown std_srvs/srv/Trigger {}
```

`isaacbot_amr_safety_node` is not started in simulation (`use_sim:=True`).

Source: `isaacbot_amr_safety/include/isaacbot_amr_safety/domain/battery_safety_policy.hpp`, `isaacbot_amr_safety/src/domain/battery_safety_policy.cpp`, `isaacbot_amr_safety/behavior_trees/isaacbot_amr_safety.xml`, `isaacbot_amr_safety/config/isaacbot_amr_safety.yaml`, `isaacbot_amr_safety/README.md`.

## Motion gates

Five independent gates stand between a velocity command and a turning wheel, from the wire inwards:

| # | Gate | Where | Behaviour |
|---|---|---|---|
| 1 | Motor contactor | Hardware | Cuts motor power when the latch is set. |
| 2 | Motion lock | `isaacbot_amr_twist_mux` / `isaacbot_amr_motion_lock_node` | Publishes `motion_lock` (`std_msgs/Bool`) at 10 Hz. `twist_mux` lock priority 200 masks every input. Locked before the first message, when either safety topic is older than `gpio_timeout` (1.0 s), when `link_healthy` is false, or when any enabled stop condition is active. A stale lock (0.5 s) also counts as locked. |
| 3 | Hardware interface `write()` | `isaacbot_amr_hardware_interface` | On any E-Stop or latched hardware fault it sends **zeros** to the motor drivers rather than going silent, so the drivers' own failsafe (`motor_failsafe_timeout_ms`, 500 ms) keeps being fed. The zeros go through each motor's command worker, so a stalled RT loop still starves the watchdog and trips it. |
| 4 | Contactor plausibility check | `ContactorMonitor` | Latch active and contactor still engaged for more than 500 ms is a latched, fatal "welded contactor" fault. Cleared only by `sw_e_stop_latch_reset`. |
| 5 | `IsMotionLocked` BT condition | Navigation stack (another repo) | Aborts a navigation tree rather than driving into a closed mux. |

Stop conditions enabled in the motion lock: HW E-Stop button, SW user E-Stop, SW motor-driver fault, latch status. `require_motor_contactor_engaged` is off (unverified on hardware).

Source: `isaacbot_amr_arch/SAFETY_CHAIN.md` §5, `isaacbot_amr_twist_mux/config/isaacbot_amr_motion_lock.yaml`, `isaacbot_amr_twist_mux/config/isaacbot_amr_twist_mux.yaml`, `isaacbot_amr_hardware_interface/include/isaacbot_amr_hardware_interface/domain/contactor_monitor.hpp` (`kDefaultContactorDropOutToleranceMs`), `isaacbot_amr_description/urdf/ardennes/ardennes_macro.urdf.xacro`.

!!! note
    With no hardware interface running there is no safety state, so the motion lock stays closed and the isaacbot accepts no velocity commands. This is intended.

The driving modes (MANUAL, ASSISTED, AUTOMATIC) and the lidar collision monitors live in `isaacbot_amr_orchestrator`, not in this repository. They are not safety functions.

## Timing budget

| Stage | Value | Source |
|---|---|---|
| `controller_manager`, `read()` / `write()` | 25 Hz | `isaacbot_amr_controller/config/wheel_01_controller.yaml` |
| `safety_status` / `safety_command_echo` publish | 20 Hz | `driver_states_update_frequency`, URDF |
| Modbus IO poll | 100 ms (10 Hz) | `safety_io_poll_period_ms`, URDF |
| Watchdog heartbeat toggle | 200 ms (5 Hz) | `safety_wdg_kick_period_ms`, URDF |
| Modbus response timeout | 150 ms | `modbus_response_timeout_ms`, URDF |
| Relay watchdog window | ~1 s (measured) | `isaacbot_amr_arch/SAFETY_CHAIN.md` §3 |
| Latch reset pulse | 100 ms | `safety_latch_reset_pulse_ms`, URDF |
| Motion lock publish / staleness / mux lock timeout | 10 Hz / 1.0 s / 0.5 s | `isaacbot_amr_twist_mux/config/` |
| `twist_mux` input timeouts | 0.3 s (Driver UI), 0.5 s (others) | `isaacbot_amr_twist_mux/config/isaacbot_amr_twist_mux.yaml` |
| `diff_drive` `cmd_vel_timeout` | 0.5 s | `isaacbot_amr_controller/config/wheel_01_controller.yaml` |
| Motor driver hardware watchdog | 500 ms | `motor_failsafe_timeout_ms`, URDF |
| Motor driver hardware watchdog on/off | on | `motor_failsafe_enabled`, URDF, optional. Off only on instrumented builds: a stalled `write()` loop then leaves the last velocity target in place, and the PLC heartbeat (its own thread) does not catch a loop-only stall. Keep wheels lifted or speeds low and the RC E-Stop within reach |
| Contactor drop-out tolerance | 500 ms | `contactor_monitor.hpp` |
| RC frame timeout / zero burst | 200 ms / 300 ms | `isaacbot_amr_crsf_teleop/config/isaacbot_amr_crsf_teleop.yaml` |

"URDF" is `isaacbot_amr_description/urdf/ardennes/ardennes_macro.urdf.xacro`.

## Trigger an E-Stop

Use any of these. The first is the only one that does not depend on software.

1. Press the hardware E-Stop button.
2. Move the RC transmitter's channel 5 switch to low.
3. Press the SW E-Stop button in the Foxglove layout or the Driver UI.
4. Call the service:

    ```bash
    ros2 service call /isaacbot/hardware_interface/sw_user_e_stop_set std_srvs/srv/Trigger
    ```

Check the result:

```bash
ros2 topic echo /isaacbot/hardware_interface/safety_status --once
```

`latch_active` should read `true` and `motor_contactor_engaged` should read `false`.

## Reset an E-Stop

The latch is set-dominant. Clear every trip source first, then reset the latch.

### Reset after a hardware button stop

1. Make sure the area around the isaacbot is clear and nothing commands motion (sticks centred, joystick released).
2. Release the hardware E-Stop button.
3. Reset the latch: press the hardware reset button, **or** move RC channel 4 to low, **or** call:

    ```bash
    ros2 service call /isaacbot/hardware_interface/sw_e_stop_latch_reset std_srvs/srv/Trigger
    ```

4. Check that `latch_active` is `false` and `motor_contactor_engaged` is `true` on `hardware_interface/safety_status`, and that `motion_lock` is `false`.

### Reset after a software E-Stop

1. Wait until the isaacbot has stopped. Centre the sticks and release every joystick.
2. Release the SW user E-Stop: RC channel 5 to high, **or** call:

    ```bash
    ros2 service call /isaacbot/hardware_interface/sw_user_e_stop_reset std_srvs/srv/Trigger
    ```

3. Reset the latch as in step 3 above.
4. Check the topics as in step 4 above.

### Reset after start-up

The latch is set after every start of the hardware interface. Wait until the hardware interface is ACTIVE, then reset the latch as above. The latch reset services refuse while the hardware interface is not ACTIVE.

### When a reset is refused

| Message or symptom | Cause | Fix |
|---|---|---|
| `E-Stop reset refused: the wheels are still turning` | A measured wheel speed is above 0.05 rad/s | Wait for the isaacbot to stop. |
| `E-Stop reset refused: largest` ... `velocity command` ... | A wheel command is above 0.4 rad/s | Stop the source publishing `cmd_vel`. For RC teleop, the stick deadband must cover the sticks' resting offset (calibrate the transmitter). |
| `... not in ACTIVE state` | Hardware interface not active | Check `ros2 control list_hardware_components`. |
| Service succeeds but the latch stays set | A trip source is still asserted (set-dominant) | Release the HW button and the SW user E-Stop first. |

Source: `isaacbot_amr_hardware_interface/src/isaacbot_amr_system/isaacbot_amr_system.cpp`.

## LED status

`isaacbot_amr_led_safety_node` (package `isaacbot_amr_safety`) picks the LED animations from the battery state and the hardware E-Stop button, and requests them from `isaacbot_amr_led`. Each bumper has four layers; a higher layer covers the ones below where its pixels are not transparent. Layer order from top: `ERROR` (0), `ALERT` (1), `INFO` (2), `STATE` (3).

!!! warning
    The `E_STOP` animation follows only `hw_e_stop_user_button`. A software E-Stop, an RC E-Stop, a watchdog trip or a held latch leaves the LEDs on `READY`. Check `hardware_interface/safety_status` or the Foxglove indicators to know whether the latch is set.

Each image below is a timeline: the top row is the first frame and each column is one LED across the bumper. `isaacbot_amr_led` stretches the image to the panel width and the animation duration. Front images are white and rear images are red; they are shown on a dark background.

| Animation (id) | Layer | Shown when | Front | Rear |
|---|---|---|---|---|
| `E_STOP` (0) | STATE | Hardware E-Stop button pressed | ![E_STOP front](assets/images/led/estop_front.png){ width="46" height="175" style="background:#222" } | ![E_STOP rear](assets/images/led/estop_rear.png){ width="46" height="175" style="background:#222" } |
| `READY` (1) | STATE | Button not pressed, no dead-man button held | ![READY front](assets/images/led/ready_front.png){ width="46" height="175" style="background:#222" } | ![READY rear](assets/images/led/ready_rear.png){ width="46" height="175" style="background:#222" } |
| `MANUAL_ACTION` (4) | STATE | Dead-man button on `joy` held (see note) | ![MANUAL_ACTION front](assets/images/led/manual_action_front.png){ width="46" height="175" style="background:#222" } | ![MANUAL_ACTION rear](assets/images/led/manual_action_rear.png){ width="46" height="175" style="background:#222" } |
| `ERROR` (2) | ERROR | Battery status `UNKNOWN`, or charging while `OVERHEAT`. Flashes over everything. | ![ERROR](assets/images/led/error.png){ width="46" height="175" style="background:#222" } | same image |
| `CHARGER_INSERTED` (9) | ALERT | Charger connected (after 0.3 s) | ![CHARGER_INSERTED front](assets/images/led/charger_inserted_front.png){ width="46" height="175" style="background:#222" } | ![CHARGER_INSERTED rear](assets/images/led/charger_inserted_rear.png){ width="46" height="175" style="background:#222" } |
| `CHARGING_BATTERY` (7) | INFO | Charging (after 2.5 s), bar shows the level | ![CHARGING_BATTERY front](assets/images/led/charging_battery_front.png){ width="50" height="175" style="background:#222" } | ![CHARGING_BATTERY rear](assets/images/led/charging_battery_rear.png){ width="50" height="175" style="background:#222" } |
| `BATTERY_CHARGED` (8) | INFO | Charging and the rounded level reads 100 % | ![BATTERY_CHARGED front](assets/images/led/battery_charged_front.png){ width="50" height="175" style="background:#222" } | ![BATTERY_CHARGED rear](assets/images/led/battery_charged_rear.png){ width="50" height="175" style="background:#222" } |
| `LOW_BATTERY` (5) | INFO | Discharging, level from 10 % to below 40 %; repeated every 30 s | ![LOW_BATTERY](assets/images/led/low_battery.png){ width="50" height="175" style="background:#222" } | same image, mirrored |
| `CRITICAL_BATTERY` (6) | INFO | Discharging, level below 10 % | ![CRITICAL_BATTERY](assets/images/led/critical_battery.png){ width="50" height="175" style="background:#222" } | same image, mirrored |
| `BATTERY_NOMINAL` (10) | INFO | Discharging, level 40 % or more (blank) | (empty) | (empty) |
| `NO_ERROR` (3) | ERROR | No battery error (blank) | (empty) | (empty) |

Thresholds: `battery.percent.threshold.critical` 0.1, `battery.percent.threshold.low` 0.4, `battery.anim_period.low` 30 s.

Source: `isaacbot_amr_safety/include/isaacbot_amr_safety/domain/led_animation_policy.hpp`, `isaacbot_amr_safety/behavior_trees/isaacbot_amr_led_safety.xml`, `isaacbot_amr_safety/config/led_safety.yaml`, `isaacbot_amr_led/config/ardennes_animations.yaml`, images from `isaacbot_amr_led/animations/ardennes/`.

!!! note
    Nothing in this repository publishes `joy`, so `MANUAL_ACTION` is not shown unless a joystick driver is added. Driving with the RC transmitter shows `READY`.

The full animation catalog and the LED services are in [Teleop and LEDs](software/teleop-and-leds.md#led-panels).

## Known gaps

From `isaacbot_amr_arch/SAFETY_CHAIN.md` §6, plus what the code shows:

- **The PLC cannot say why it tripped.** `SafetyStatus.latch_cause` exists but reads `LATCH_CAUSE_UNKNOWN` until the PLC program latches the cause into readable coils.
- **The command-side reset deadband is still wide.** `velocity_command_zero_tolerance` is 0.4 rad/s. It was set that way because the host wheel PIDs' frozen integral kept the command above zero while motion was inhibited. Since the DCC1020 swap there are no host wheel PIDs, so `diff_drive`'s zero command reaches the hardware as exactly 0 and that reason no longer holds. The tolerance has not been lowered yet. The measured-velocity check (0.05 rad/s) closes the actual hazard in the meantime.
- **The DCC1020 hardware E-Stop input is not wired into the safety chain.** It is always enabled but still jumpered as shipped. Wiring it is an [open item](open-items.md).
- **The new DCC1020 fault bits are diagnostics only.** Over-temperature, over-current, over-voltage, stall and bad-power show as `fault_flag.*` in diagnostics, but the `FaultFlag` message carries only `emergency_stop` and `motor_setup_fault`.
- **No end-to-end test** that an asserted `motion_lock` stops `cmd_vel` at the mux.
- **The lidar guards are obstacle avoidance, not protection.** One 2D scan slice misses low and overhanging obstacles. MANUAL, RC and Foxglove bypass them. No measured stop distance yet.
- **`ros2_control`'s GPIO mechanism is not used** for the safety IO (deliberate).
- **`read()` / `write()` always return `OK`.** A dead safety link reaches diagnostics and the topics, but `controller_manager` cannot deactivate on it.
- **LEDs do not show a software E-Stop or a held latch** (see [LED status](#led-status)).
- **The software motor-driver fault input is not asserted at runtime** (see the note in [Safety PLC signals](#safety-plc-signals)).

Full text: [SAFETY_CHAIN.md](https://github.com/isaacbotsrl/isaacbot_amr_ros/blob/main/isaacbot_amr_arch/SAFETY_CHAIN.md).

## Transport and handling

Lifting points, towing, free-wheeling with power off, shipping, storage and battery handling are **TBD**. Until they are documented, power the isaacbot down and press the hardware E-Stop before you lift or move it by hand.

## Further reading

- [isaacbot_amr_safety README](https://github.com/isaacbotsrl/isaacbot_amr_ros/blob/main/isaacbot_amr_safety/README.md)
- [isaacbot_amr_twist_mux README](https://github.com/isaacbotsrl/isaacbot_amr_ros/blob/main/isaacbot_amr_twist_mux/README.md)
- [isaacbot_amr_hardware_interface README](https://github.com/isaacbotsrl/isaacbot_amr_ros/blob/main/isaacbot_amr_hardware_interface/README.md)
- [isaacbot_amr_crsf_teleop README](https://github.com/isaacbotsrl/isaacbot_amr_ros/blob/main/isaacbot_amr_crsf_teleop/README.md)
