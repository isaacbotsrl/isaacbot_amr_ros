# isaacbot_amr_description

URDF/xacro model of Ardennes: links, joints, meshes, sensors (IMU, lidar), the simulated
battery, and the `<ros2_control>` hardware components the driver stack runs on.

## Model files

```
urdf/ardennes.urdf.xacro              entry point (xacro args below)
urdf/ardennes/ardennes_macro.urdf.xacro  robot macro: body, wheels, sensors, <ros2_control> system
urdf/ardennes/base.urdf.xacro         chassis (base_link / body_link / base_footprint)
urdf/common/wheel.urdf.xacro          wheel link + continuous joint (fl, fr, rl, rr)
urdf/common/imu.urdf.xacro            imu_link, Gazebo IMU sensor, <ros2_control> sensor component
urdf/common/lidar.urdf.xacro          lidar_link and the Gazebo lidar
urdf/common/battery.urdf.xacro        Gazebo LinearBatteryPlugin
urdf/common/gazebo_system.urdf.xacro  gz_ros2_control plugin (simulation only)
meshes/ardennes/, meshes/wheel_01/    STL meshes
rviz/isaacbot.rviz                       RViz layout
```

`urdf/ardennes.urdf.xacro` arguments: `use_sim`, `namespace`, `wheel_config_file`
(`config/wheel_01.yaml`), `battery_config_file` (`config/battery.yaml`),
`controller_config_file` (required with `use_sim`), `imu_xyz`/`imu_rpy`, `lidar_xyz`/`lidar_rpy`.

## ros2_control components

With `use_sim:=false` the URDF declares two hardware components. Each one runs its own node
inside `controller_manager`, named after the component in the controller manager's namespace
(`/isaacbot/isaacbot_amr_system_node`, `/isaacbot/isaacbot_amr_imu`). The names carry no namespace prefix: a
`isaacbot/` prefix would make the node `/isaacbot/isaacbot_amr_isaacbot_amr_system_node`.

| Component (`<ros2_control name>`) | Type | Plugin | Exports |
|-----------------------------------|------|--------|---------|
| `isaacbot_amr_system_node` | system | `isaacbot_amr_hardware_interface/ArdennesSystem` | velocity command + position/velocity/effort state per wheel joint; drives the Phidget DCC1020 motor controllers (on-board speed loop) and talks Modbus TCP to the safety controller (E-Stop, GPIO) at `192.168.88.11:502` |
| `isaacbot_amr_imu` | sensor | `isaacbot_amr_hardware_interface/PhidgetImuSensor` (Phidgets Spatial + Madgwick filter) | `imu/orientation.*`, `imu/angular_velocity.*`, `imu/linear_acceleration.*` |

The hardware parameters (Modbus address, timeouts, gear ratio, encoder resolution, …) are the
`<param>` tags in `urdf/ardennes/ardennes_macro.urdf.xacro` and `urdf/common/imu.urdf.xacro`. The
DCC1020 on-board velocity loop settings (`velocity_kp/ki/kd`, `velocity_deadband_rpm`,
`motor_acceleration_rpm_s`, surge/stall/failsafe current, `motor_inductance`) are optional and still TBD (commented out: board defaults);
`motor_failsafe_braking_enabled` defaults to true. The DCC1000 parameters `motor_acceleration` and
`motor_supply_voltage` were removed and are rejected by the hardware interface, as is
`motor_data_interval_ms` (the feedback always runs at the board's minimum data interval). The CAD-derived
base mass still models four DCC1000s (78.8 g each, ASSUMPTION), not the DCC1020s now fitted. With
`use_sim:=true`, the system uses `gz_ros2_control/GazeboSimSystem` and carries the IMU interfaces
(fed by the Gazebo IMU sensor), and `isaacbot_amr_imu` has no hardware. gz_ros2_control doesn't pass its
namespace on to the component node, so in simulation it is `/isaacbot_amr_system_node`.

## Frame Conventions

Standard ROS conventions ([REP-103](https://ros.org/reps/rep-0103.html)) throughout.
For `base_link` / `body_link`:

| Axis | Direction |
|------|-----------|
| `+x` / `-x` | forward / backward |
| `+y` / `-y` | left / right |
| `+z` / `-z` | up / down |
| `+yaw` (`angular.z`) | counter-clockwise seen from above, i.e. turning left |

This puts the four wheels at:

| Prefix | x | y | Corner |
|--------|---|---|--------|
| `fl` | `+wheelbase/2` | `+wheel_separation/2` | front-left |
| `fr` | `+wheelbase/2` | `-wheel_separation/2` | front-right |
| `rl` | `-wheelbase/2` | `+wheel_separation/2` | rear-left |
| `rr` | `-wheelbase/2` | `-wheel_separation/2` | rear-right |

All four wheel joints spin about **`+y`** (`<axis xyz="0 1 0"/>`). That sign is
deliberate and the same for every wheel: with the contact patch below the hub,
a positive joint velocity drives the isaacbot **forward** (`+x`). No wheel needs a
negated axis. `isaacbot_amr_drive_controller`'s `left_wheel_names` are correspondingly the
`+y` pair (`rl`, `fl`) and `right_wheel_names` the `-y` pair (`rr`, `fr`).

`base_footprint` is the root (REP 105): the ground projection of `base_link`. The odometry
filters publish `odom → base_footprint`, so in 2D mode `odom` stays on the ground plane.
`base_link` sits `tyre_radius - wheel_mount_point_z` above it and coincides with `body_link`
(the chassis centre).

## Config Files

- `wheel_01.yaml` - Isaacbot 4WD 13-inch wheel (`wheel_radius` 0.1651 m, 0.33 m diameter) configuration, including `wheelbase` and
  `wheel_separation`. Both are **full centre-to-centre** distances (front-to-rear and
  left-to-right respectively); the URDF halves them to get each mount point. This is the
  single source of truth for wheel geometry; `isaacbot_amr_controller/config/wheel_01_controller.yaml`
  must match it (checked by `isaacbot_amr_controller/test/test_wheel_geometry.py`).

  Note that `wheel_separation_multiplier` in that controller config is **not** geometry -
  it is the usual empirical skid-steer correction (a 4-wheel skid-steer's effective track
  exceeds its physical track), so it is not expected to match anything here. It is measured
  with `isaacbot_amr_controller`'s `wheel_odom_calibration` tool (see "Drive-train tuning" in
  `isaacbot_amr_controller/README.md`); the controller config records the current value and where
  it came from.

- `battery.yaml` - Simulated battery (Gazebo `LinearBatteryPlugin`) configuration.

## Launch Files

- `isaacbot_amr_load_urdf.launch.py` - generates the URDF with xacro and runs `robot_state_publisher`
  (node `isaacbot_amr_state_publisher_node`, publishing `robot_description` and TF with the
  `<namespace>/` frame prefix). `controller_config_path` is required: it is embedded in the URDF
  for `gz_ros2_control`, and `isaacbot_amr_controller.launch.py` supplies it.

  | Argument | Default | Description |
  |----------|---------|-------------|
  | `controller_config_path` | (required) | Controller config embedded in the URDF. |
  | `namespace` | `$ISAACBOT_AMR_SYSTEM_NAMESPACE`, else empty | Namespace and TF frame prefix. |
  | `robot_model` | `$ROBOT_MODEL_NAME`, else `ardennes` | Robot model. |
  | `use_sim` | `False` | Build the simulation variant (`gz_ros2_control`). |
  | `wheel_type` | `wheel_01` | `wheel_01` or `custom`; selects `config/<wheel_type>.yaml`. |
  | `wheel_config_path` | `config/<wheel_type>.yaml` | Wheel geometry file. |
  | `publish_robot_state` | `True` | Start `robot_state_publisher`. |

- `isaacbot_amr_rviz.launch.py` - RViz with `rviz/isaacbot.rviz`. `<namespace>` in the config is
  substituted.

  | Argument | Default | Description |
  |----------|---------|-------------|
  | `rviz_config` | `rviz/isaacbot.rviz` | RViz config file. |
  | `namespace` | `$ISAACBOT_AMR_SYSTEM_NAMESPACE`, else empty | Namespace used in the config. |
  | `use_sim` | `False` | Use simulation time. |
