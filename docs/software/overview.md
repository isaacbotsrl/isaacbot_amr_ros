# Software overview

The Ardennes platform software is the `isaacbot_amr_ros` repository: a ROS 2 workspace that drives the
wheels, reads the IMU and the safety PLC, arbitrates velocity commands, fuses odometry and runs
the battery, LED, teleop and diagnostics nodes. This page shows how it is deployed, what each
package does, and how to build and start it.

## Runtime environment

| Item | Value |
|------|-------|
| ROS 2 distribution | `lyrical` |
| Middleware (RMW) | `rmw_zenoh_cpp`, ROS domain `0` |
| Zenoh router | Runs in its own container, `ardennes-zenoh-router`, on TCP port `7447` |
| Platform container | `ardennes-platform`. Its image is built from `rover_docker/rovera1_app` (separate repository) |
| Device management | balenaCloud. Device variables such as `ISAACBOT_AMR_SYSTEM_USE_GPS` reach the launch files as environment variables; the container's `start.sh` normalizes them |
| Isaacbot LAN address | `192.168.1.201` (default target of the PC setup script) |
| Robot namespace | `isaacbot` (`ISAACBOT_AMR_SYSTEM_NAMESPACE`) |
| Controller computer (CPU, OS) | **TBD** |

Source: `README.md`, `isaacbot_amr_scripts/setup_isaacbot_amr_pc.sh`, `isaacbot_amr_scripts/README.md`,
`isaacbot_amr_bringup/launch/isaacbot_amr_bringup.launch.py`, `isaacbot_amr_metapackage/README.md`.

The platform container runs two launch files: `isaacbot_amr_bringup.launch.py` (the driver stack) and
`isaacbot_amr_web_bridges.launch.py` (Foxglove bridge, rosbridge, rosapi). The container definition and
`start.sh` live in `rover_docker`, not in this repository.

!!! note "Three containers make a isaacbot"
    A complete isaacbot runs three application repositories, one container each. This manual covers
    only the platform.

    | Repository | Container | Role |
    |------------|-----------|------|
    | [`isaacbot_amr_ros`](https://github.com/isaacbotsrl/isaacbot_amr_ros) | `ardennes-platform` | This platform: drive, safety IO, localization, power, LEDs, teleop, diagnostics. |
    | [`isaacbot_amr_sensors`](https://github.com/isaacbotsrl/isaacbot_amr_sensors) | `ardennes-sensors` | Sensor payload: GNSS and lidar drivers. Publishes `gps/fix`, `scan` and `rslidar_points`. |
    | [`isaacbot_amr_orchestrator`](https://github.com/isaacbotsrl/isaacbot_amr_orchestrator) | `ardennes-orchestrator` | Autonomy: Nav 2 and mission supervision. Publishes `nav_cmd_vel_stamped`. |

    Source: `README.md`, `isaacbot_amr_bringup/README.md`.

## Runtime node graph

The main nodes started by `isaacbot_amr_bringup.launch.py` and `isaacbot_amr_web_bridges.launch.py`. Every node
runs in the `isaacbot` namespace except the web bridges. Topic names are relative to that namespace.

```mermaid
flowchart LR
  subgraph HW["Hardware"]
    PLC["Safety PLC (Modbus TCP)"]
    MOT["4x DCC1020 motor controllers<br/>(on-board speed loop)"]
    IMU["Phidget IMU"]
    BMS["BMS bridge (UDP)"]
    LEDB["LED boards (UDP)"]
    RC["ELRS receiver (UDP via RUTX11)"]
  end

  subgraph CM["controller_manager process"]
    HWI["ArdennesSystem + PhidgetImuSensor<br/>(isaacbot_amr_hardware_controller)"]
    DD["isaacbot_amr_drive_controller"]
    JSB["isaacbot_amr_joint_state_broadcaster"]
    IMUB["isaacbot_amr_imu_broadcaster"]
  end

  PLC <--> HWI
  MOT <--> HWI
  IMU --> HWI
  DD --> HWI
  HWI --> JSB
  HWI --> IMUB

  RCT["isaacbot_amr_crsf_teleop_node"]
  FR["isaacbot_amr_command_freshness_node"]
  NAV["Nav 2 (orchestrator)"]
  MUX["isaacbot_amr_twist_mux_node"]
  LOCK["isaacbot_amr_motion_lock_node"]

  RC --> RCT
  RCT -- teleop_elrs_cmd_vel_stamped --> MUX
  FR -- teleop_driver_interface_cmd_vel_fresh_stamped --> MUX
  NAV -- nav_cmd_vel_stamped --> MUX
  HWI -- "hardware_interface/safety_*" --> LOCK
  LOCK -- motion_lock --> MUX
  MUX -- cmd_vel --> DD

  EKF["isaacbot_amr_ekf_node"]
  DD -- odometry/wheels --> EKF
  IMUB -- imu/data --> EKF
  EKF -- "odom, TF odom to base_footprint" --> NAV

  SAF["isaacbot_amr_safety_node"]
  LSAF["isaacbot_amr_led_safety_node"]
  BAT["isaacbot_amr_battery_node"]
  LED["isaacbot_amr_led_controller + isaacbot_amr_led_driver"]
  BMS --> BAT
  BAT -- battery/battery_status --> SAF
  BAT -- battery/battery_status --> LSAF
  SAF -- "sw_user_e_stop_set (srv)" --> HWI
  LSAF -- "led/set_animation (srv)" --> LED
  LED --> LEDB

  DIAG["isaacbot_amr_diag_manager_node"]
  AGG["isaacbot_amr_diagnostic_aggregator"]
  DIAG -- diagnostics --> AGG
  WEB["isaacbot_amr_foxglove_bridge, isaacbot_amr_rosbridge_websocket, rosapi"]
  AGG -- diagnostics_agg --> WEB
```

Source: `isaacbot_amr_bringup/launch/isaacbot_amr_bringup.launch.py`, `isaacbot_amr_bringup/launch/isaacbot_amr_web_bridges.launch.py`,
`isaacbot_amr_controller/config/wheel_01_controller.yaml`, `isaacbot_amr_twist_mux/config/isaacbot_amr_twist_mux.yaml`,
the package launch files. The [ROS 2 API](ros-api.md) page lists every topic and service.

### Startup order

1. `isaacbot_amr_bringup` checks `ROBOT_HW_CONFIG_CORRECT`. If it is not `true`, nothing starts.
2. `isaacbot_amr_controller` (robot description, `controller_manager`, controllers) and
   `isaacbot_amr_diag_manager` start at once.
3. The controllers are spawned in order: joint state broadcaster, drive controller, IMU
   broadcaster. A failed spawner shuts the launch down.
4. When the IMU broadcaster is active, `isaacbot_amr_battery`, `isaacbot_amr_led`, `isaacbot_amr_safety`,
   `isaacbot_amr_localization`, `isaacbot_amr_crsf_teleop` and `isaacbot_amr_twist_mux` start. If that event does not
   come within `controllers_ready_timeout` (20.0 s), they start anyway with a warning.

Source: `isaacbot_amr_bringup/launch/isaacbot_amr_bringup.launch.py`, `isaacbot_amr_controller/launch/isaacbot_amr_controller.launch.py`.

## Packages

| Package | Layer / role | What it does | README |
|---------|--------------|--------------|--------|
| `isaacbot_amr_bringup` | Bringup | Top-level launch for real hardware, plus the web bridges launch. | [README](https://github.com/isaacbotsrl/isaacbot_amr_ros/blob/main/isaacbot_amr_bringup/README.md) |
| `isaacbot_amr_gazebo` | Simulation | Gazebo Sim bringup with `gz_ros2_control`, the same controllers, EKF and twist_mux. | [README](https://github.com/isaacbotsrl/isaacbot_amr_ros/blob/main/isaacbot_amr_gazebo/README.md) |
| `isaacbot_amr_gazebo_plugins` | Simulation | Gazebo GUI plugins: the Isaacbot Safety panel (HW / SW E-Stop, latch reset) for `isaacbot_amr_gazebo`'s `sim_safety_plc`. | [README](https://github.com/isaacbotsrl/isaacbot_amr_ros/blob/main/isaacbot_amr_gazebo_plugins/README.md) |
| `isaacbot_amr_world` | Simulation | Gazebo worlds. | [README](https://github.com/isaacbotsrl/isaacbot_amr_ros/blob/main/isaacbot_amr_world/README.md) |
| `isaacbot_amr_description` | Model | URDF/xacro: links, joints, meshes, sensors, `<ros2_control>` components. | [README](https://github.com/isaacbotsrl/isaacbot_amr_ros/blob/main/isaacbot_amr_description/README.md) |
| `isaacbot_amr_metapackage` | Build | Selects `isaacbot_amr_bringup` or `isaacbot_amr_gazebo` by `ISAACBOT_AMR_ROS_BUILD_TYPE`; holds the `.repos` lists. | [README](https://github.com/isaacbotsrl/isaacbot_amr_ros/blob/main/isaacbot_amr_metapackage/README.md) |
| `isaacbot_amr_hardware_interface` | Hardware (ros2_control plugins) | `ArdennesSystem` (wheel motors, safety PLC) and `PhidgetImuSensor`. | [README](https://github.com/isaacbotsrl/isaacbot_amr_ros/blob/main/isaacbot_amr_hardware_interface/README.md) |
| `isaacbot_amr_controller` | Control | Controller configuration and launch; drive-train tuning tools; `SeededPidController` (built, no longer spawned since the DCC1020 on-board speed loop). | [README](https://github.com/isaacbotsrl/isaacbot_amr_ros/blob/main/isaacbot_amr_controller/README.md) |
| `isaacbot_amr_twist_mux` | Control | `twist_mux` arbitration, `isaacbot_amr_motion_lock_node`, `isaacbot_amr_command_freshness_node`. | [README](https://github.com/isaacbotsrl/isaacbot_amr_ros/blob/main/isaacbot_amr_twist_mux/README.md) |
| `isaacbot_amr_crsf_teleop` | Teleop | CRSF (ExpressLRS) RC teleop, software E-Stop switches, RC link failsafe. | [README](https://github.com/isaacbotsrl/isaacbot_amr_ros/blob/main/isaacbot_amr_crsf_teleop/README.md) |
| `isaacbot_amr_localization` | Localization | `robot_localization` EKFs: wheels + IMU, optionally GPS. | [README](https://github.com/isaacbotsrl/isaacbot_amr_ros/blob/main/isaacbot_amr_localization/README.md) |
| `isaacbot_amr_gps_heading` | Localization | ENU heading from GNSS course, for `navsat_transform_node`. | [README](https://github.com/isaacbotsrl/isaacbot_amr_ros/blob/main/isaacbot_amr_gps_heading/README.md) |
| `isaacbot_amr_safety` | Safety supervision | Behavior-tree supervision: battery E-Stop, shutdown, LED animation choice. | [README](https://github.com/isaacbotsrl/isaacbot_amr_ros/blob/main/isaacbot_amr_safety/README.md) |
| `isaacbot_amr_battery` | Power | Decodes BMS telemetry received over UDP and publishes the battery state. | [README](https://github.com/isaacbotsrl/isaacbot_amr_ros/blob/main/isaacbot_amr_battery/README.md) |
| `isaacbot_amr_led` | Indication | SK9822 bumper LED panels: animation controller and driver. | [README](https://github.com/isaacbotsrl/isaacbot_amr_ros/blob/main/isaacbot_amr_led/README.md) |
| `isaacbot_amr_diag_manager` | Diagnostics | Computer health node and the diagnostic aggregator. | [README](https://github.com/isaacbotsrl/isaacbot_amr_ros/blob/main/isaacbot_amr_diag_manager/README.md) |
| `isaacbot_amr_msgs` | Interfaces | Custom messages and services. | [README](https://github.com/isaacbotsrl/isaacbot_amr_ros/blob/main/isaacbot_amr_msgs/README.md) |
| `isaacbot_amr_utils` | Utilities | Header-only C++ helpers and Python launch helpers. | [README](https://github.com/isaacbotsrl/isaacbot_amr_ros/blob/main/isaacbot_amr_utils/README.md) |
| `isaacbot_amr_transport` | Drivers | UDP driver (a `transport_drivers` fork) and a Modbus TCP client library. | [README](https://github.com/isaacbotsrl/isaacbot_amr_ros/blob/main/isaacbot_amr_transport/README.md) |
| `isaacbot_amr_modbus` | Library | Vendored Modbus library for modern C++. | [README](https://github.com/isaacbotsrl/isaacbot_amr_ros/blob/main/isaacbot_amr_modbus/README.md) |
| `isaacbot_amr_arch` | Docs (not a ROS package) | Firmware architecture and the safety chain. | [README](https://github.com/isaacbotsrl/isaacbot_amr_ros/blob/main/isaacbot_amr_arch/README.md) |
| `isaacbot_amr_foxglove` | Tools (not a ROS package) | Foxglove dashboard layout for the `isaacbot` namespace. | [README](https://github.com/isaacbotsrl/isaacbot_amr_ros/blob/main/isaacbot_amr_foxglove/README.md) |
| `isaacbot_amr_scripts` | Tools (not a ROS package) | Development PC setup (`setup_isaacbot_amr_pc.sh`). | [README](https://github.com/isaacbotsrl/isaacbot_amr_ros/blob/main/isaacbot_amr_scripts/README.md) |
| `rover_platform_mbse` | MBSE (not a ROS package) | MATLAB/Simulink architecture, requirements and traceability. | [README](https://github.com/isaacbotsrl/isaacbot_amr_ros/blob/main/rover_platform_mbse/README.md) |

Source: `README.md` and each package README.

Most packages with nodes follow a Clean Architecture layout (`domain/`, `application/`,
`infrastructure/`), where `domain` code has no ROS dependency.

## Getting started

### Create the workspace

```bash
mkdir -p ~/ros2_ws/ardennes
cd ~/ros2_ws/ardennes
git clone -b master https://github.com/isaacbotsrl/isaacbot_amr_ros.git src/isaacbot_amr_ros
```

### Set the environment

The setup script writes a `isaacbot-pc setup` block into `~/.bashrc`. It sets up ROS, the workspace
overlay, the `ISAACBOT_AMR_*` variables and the Zenoh client settings for the isaacbot. It is safe to run
again.

```bash
src/isaacbot_amr_ros/isaacbot_amr_scripts/setup_isaacbot_amr_pc.sh
source ~/.bashrc
```

Or set the variables by hand:

```bash
export ROS_DISTRO=lyrical
export ISAACBOT_AMR_SYSTEM_NAMESPACE=isaacbot
export ISAACBOT_AMR_ROS_BUILD_TYPE=hardware     # real isaacbot
# export ISAACBOT_AMR_ROS_BUILD_TYPE=simulation # Gazebo
```

| Variable | Used by | Effect |
|----------|---------|--------|
| `ROS_DISTRO` | build | ROS 2 distribution (`lyrical`). |
| `ISAACBOT_AMR_ROS_BUILD_TYPE` | `isaacbot_amr_metapackage`, `vcs import` | `hardware` builds `isaacbot_amr_bringup`; `simulation` builds `isaacbot_amr_gazebo`. |
| `ISAACBOT_AMR_SYSTEM_NAMESPACE` | all launch files | Default `namespace` argument. The isaacbot uses `isaacbot`. |
| `ISAACBOT_AMR_SYSTEM_USE_GPS` | `isaacbot_amr_bringup`, `isaacbot_amr_gazebo` | `true` adds GPS fusion (dual EKF). Default `false`. |
| `RMW_IMPLEMENTATION` | all nodes | `rmw_zenoh_cpp` (set by the setup script). |
| `ZENOH_CONFIG_OVERRIDE` | all nodes | Client mode to `tcp/<isaacbot-ip>:7447` (set by the setup script). |

Source: `README.md`, `isaacbot_amr_scripts/README.md`, `isaacbot_amr_bringup/README.md`,
`isaacbot_amr_localization/launch/isaacbot_amr_localization.launch.py`.

!!! warning "The PC joins the real isaacbot's graph"
    With the setup block in `~/.bashrc`, every node started on the PC is a Zenoh client of the
    isaacbot's router. Use `isaacbot_amr_gazebo/scripts/isaacbot_amr_sim.sh` for the simulation: it clears
    `ZENOH_CONFIG_OVERRIDE` and runs a local router. See [Simulation](simulation.md).

### Get the dependencies and build

```bash
vcs import src < src/isaacbot_amr_ros/isaacbot_amr_metapackage/${ISAACBOT_AMR_ROS_BUILD_TYPE}_deps.repos

sudo apt install usbutils plocate
# Phidget22 SDK >= 1.26 (DCC1020 + IMU) from the Phidgets apt repo; there is no rosdep key for it
curl -fsSL https://www.phidgets.com/downloads/setup_linux | sudo -E bash -
sudo apt install libphidget22 libphidget22-dev
sudo rosdep init
rosdep update --rosdistro $ROS_DISTRO
rosdep install --from-paths src -y -i
```

Real isaacbot only, build and install the profiler library:

```bash
cd src/rover_cppuprofile
cmake -Bbuild . -DPROFILE_ENABLED=ON
cmake --build build
cd build
sudo make install
cd ../../..
```

Then, for both build types:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --symlink-install --packages-up-to isaacbot_amr_metapackage \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
source install/setup.bash
```

### Run

```bash
# Real isaacbot (inside the platform container, or directly)
ros2 launch isaacbot_amr_bringup isaacbot_amr_bringup.launch.py
ros2 launch isaacbot_amr_bringup isaacbot_amr_web_bridges.launch.py

# Simulation
src/isaacbot_amr_ros/isaacbot_amr_gazebo/scripts/isaacbot_amr_sim.sh
```

### Test

The build above turns testing off, so rebuild the package under test first. Run node tests one
at a time: parallel workers make them flaky under Zenoh.

```bash
colcon build --symlink-install --packages-select <pkg>
colcon test --packages-select <pkg> --parallel-workers 1
colcon test-result --all
```

Source: `README.md`.

## Bringup launch arguments

`ros2 launch isaacbot_amr_bringup isaacbot_amr_bringup.launch.py [arg:=value ...]`

| Argument | Default | Description |
|----------|---------|-------------|
| `namespace` | `$ISAACBOT_AMR_SYSTEM_NAMESPACE`, else empty | Namespace of every node. |
| `log_level` | `INFO` | `DEBUG`, `INFO`, `WARN`, `ERROR` or `FATAL`, passed to every package. |
| `use_gps` | `$ISAACBOT_AMR_SYSTEM_USE_GPS`, else `false` | `true`: wheels + IMU + GPS (dual EKF). `false`: wheels + IMU. |
| `common_dir_path` | empty | Directory with per-package config overrides (`<dir>/<package>/config/...`). |
| `disable_manager` | `False` | `True` skips `isaacbot_amr_safety`. |
| `exit_on_wrong_hw` | `false` | Exit instead of idling when `ROBOT_HW_CONFIG_CORRECT` is not `true`. |
| `controllers_ready_timeout` | `20.0` | Seconds to wait for the controllers before starting the rest anyway. |

| Environment variable | Default | Effect |
|----------------------|---------|--------|
| `ROBOT_MODEL_NAME` / `ROBOT_SERIAL_NO` / `ROBOT_VERSION` | `ardennes` / `A1-2026-01` / `1.0` | Shown in the startup banner. |
| `ROBOT_HW_CONFIG_CORRECT` | `true` | Gate for starting the driver stack. |
| `SYSTEM_BUILD_VERSION` | `v1.0.0` | Compared with the minimum OS version `v1.0.0`; a warning if older. |

Source: `isaacbot_amr_bringup/launch/isaacbot_amr_bringup.launch.py`.

The GPS and lidar drivers are not started by `isaacbot_amr_bringup`. They belong to the sensor payload
(`isaacbot_amr_sensors`).
