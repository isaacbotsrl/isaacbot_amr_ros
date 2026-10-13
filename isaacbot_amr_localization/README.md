# isaacbot_amr_localization

Fuses the isaacbot's wheel odometry and IMU (and optionally the RUTX11 GPS) with
`robot_localization` EKFs. The `ISAACBOT_AMR_SYSTEM_USE_GPS` environment variable (the `fuse_gps` argument)
selects between two modes:

| Mode | Nodes | Transforms |
|------|-------|------------|
| `ISAACBOT_AMR_SYSTEM_USE_GPS=false` (default) | `isaacbot_amr_ekf_node`: wheels + IMU yaw rate | `<ns>/odom → <ns>/base_footprint` |
| `ISAACBOT_AMR_SYSTEM_USE_GPS=true` | `isaacbot_amr_ekf_node` (unchanged), `isaacbot_amr_ekf_global_node`: wheels + IMU yaw rate + GPS position, `isaacbot_amr_gps_heading_node`, `isaacbot_amr_navsat_transform_node` | also `<ns>/map → <ns>/odom`, only with `ISAACBOT_AMR_SYSTEM_GPS_MAP_TF=true` |

With GPS, `odom` stays continuous for local control, and GPS corrections show up only in
`map → odom`. The GPS driver and its fix diagnostics (`gps/fix`) are the sensor payload
(`isaacbot_amr_sensors/isaacbot_amr_gps`, container `rover-a1-sensors`). The ENU heading (`gps/heading_imu`)
comes from `isaacbot_amr_gps_heading_node` (package `isaacbot_amr_gps_heading`), started here in GPS mode.

> By default (`ISAACBOT_AMR_SYSTEM_GPS_MAP_TF=false`, `publish_global_tf:=false`) the global EKF
> publishes `odometry/global` but does not broadcast `map → odom`, which is left to AMCL
> (`isaacbot_amr_orchestrator/isaacbot_amr_navigation/launch/localization.launch.py`) or SLAM. With
> `ISAACBOT_AMR_SYSTEM_GPS_MAP_TF=true` it broadcasts `map → odom` itself; do not run AMCL or SLAM
> alongside it then.

## Interfaces

### `isaacbot_amr_ekf_node` (both modes)

| Direction | Name | Type |
|-----------|------|------|
| sub | `odometry/wheels` | `nav_msgs/Odometry` (from `isaacbot_amr_controller`'s `isaacbot_amr_drive_controller`) |
| sub | `imu/data` | `sensor_msgs/Imu` (from `isaacbot_amr_controller`'s `isaacbot_amr_imu_broadcaster`) |
| pub | `odom` | `nav_msgs/Odometry` (the EKF's `odometry/filtered`, remapped) |
| pub | `/tf` | `<namespace>/odom → <namespace>/base_footprint` (`tf_prefix` = namespace) |
| srv | `localization/set_pose`, `localization/enable`, `localization/toggle` | robot_localization services |
| pub | `diagnostics` | robot_localization status |

### `isaacbot_amr_ekf_global_node` (GPS mode)

| Direction | Name | Type |
|-----------|------|------|
| sub | `odometry/wheels`, `imu/data` | as above |
| sub | `odometry/gps` | `nav_msgs/Odometry` from `isaacbot_amr_navsat_transform_node` (X/Y fused) |
| pub | `odometry/global` | `nav_msgs/Odometry` in `<namespace>/map` |
| pub | `/tf` | `<namespace>/map → <namespace>/odom` |
| srv | `localization/global/{set_pose,enable,toggle}` | robot_localization services |

### `isaacbot_amr_gps_heading_node` (GPS mode)

| Direction | Name | Type |
|-----------|------|------|
| sub | `gps/fix` | `sensor_msgs/NavSatFix` (sensor payload) |
| sub | `odom` | `isaacbot_amr_ekf_node` output |
| pub | `gps/heading_imu` | `sensor_msgs/Imu`, ENU yaw of `<namespace>/base_link`, once aligned |
| srv | `gps/reset_heading_alignment` | `std_srvs/Trigger` |

Parameters are in the `isaacbot_amr_gps_heading_node:` section of `config/rel_localization_with_gps.yaml`;
see `isaacbot_amr_gps_heading/README.md` for the alignment.

### `isaacbot_amr_navsat_transform_node` (GPS mode)

| Direction | Name | Type |
|-----------|------|------|
| sub | `gps/fix` | `sensor_msgs/NavSatFix` (`isaacbot_amr_gps_driver`, sensor payload) |
| sub | `gps/heading_imu` | `sensor_msgs/Imu`, ENU heading from `isaacbot_amr_gps_heading_node` (published once aligned) |
| sub | `odometry/global` | global EKF output |
| pub | `odometry/gps` | GPS position in the map frame |
| pub | `gps/filtered` | `sensor_msgs/NavSatFix` of the filtered pose |
| srv | `localization/datum` | robot_localization `SetDatum` |

The filters run at 50 Hz in 2D mode (`two_d_mode: true`).

## Config Files

- `config/rel_localization.yaml`: `isaacbot_amr_ekf_node`, which fuses the IMU (`imu/data`) and wheel
  odometry (`odometry/wheels`). The output is published on `odom`.
  `print_diagnostics` must stay `true`. robot_localization always reports the
  `odometry/filtered topic status` frequency diagnostic but only counts publishes when it is
  enabled, so with `false` that status is a permanent false ERROR ("No events recorded").
- `config/rel_localization_with_gps.yaml`: the same `isaacbot_amr_ekf_node`, plus `isaacbot_amr_ekf_global_node`
  (`world_frame: map`, `odom1: odometry/gps` X/Y) and `isaacbot_amr_navsat_transform_node`
  (`yaw_offset: 0`, `magnetic_declination_radians: 0`, because the heading already arrives in
  ENU; `zero_altitude: true`).

## Launch Files

- `isaacbot_amr_localization.launch.py`: starts `isaacbot_amr_ekf_node`, and in GPS mode also
  `isaacbot_amr_ekf_global_node` and `isaacbot_amr_navsat_transform_node`.

| Argument | Default | Description |
|----------|---------|-------------|
| `use_ekf` | `False` | Start the EKFs. `isaacbot_amr_bringup` and `isaacbot_amr_gazebo` pass `True`. |
| `fuse_gps` | `$ISAACBOT_AMR_SYSTEM_USE_GPS`, else `false` | GPS mode (`true`/`1`/`yes`/`on`, any case). Selects the `_with_gps` config. `isaacbot_amr_bringup` and `isaacbot_amr_gazebo` pass their `use_gps` (default `$ISAACBOT_AMR_SYSTEM_USE_GPS`, else `false`). |
| `publish_global_tf` | `$ISAACBOT_AMR_SYSTEM_GPS_MAP_TF`, else `false` | GPS mode only: `isaacbot_amr_ekf_global_node` broadcasts `map → odom` (`true`/`1`/`yes`/`on`, any case). Overrides the config's `publish_tf`. `false` (default) leaves `map → odom` to SLAM/AMCL; `odometry/global` is still published. |
| `namespace` | `$ISAACBOT_AMR_SYSTEM_NAMESPACE`, else empty | Namespace and TF prefix. |
| `localization_mode` | `rel` | `rel`: relative to the start pose; `enu`: East-North-Up orientation. Selects `config/<mode>_localization[_with_gps].yaml`. |
| `localization_config_path` | `config/<mode>_localization[_with_gps].yaml` | Explicit EKF config. |
| `common_dir_path` | empty | If set, the default config is read from `<common_dir_path>/isaacbot_amr_localization/config/`. |
| `use_sim` | `False` | Use simulation time. |
| `log_level` | `INFO` | Logging level. |

```bash
ros2 launch isaacbot_amr_localization isaacbot_amr_localization.launch.py use_ekf:=True
ISAACBOT_AMR_SYSTEM_USE_GPS=true ros2 launch isaacbot_amr_localization isaacbot_amr_localization.launch.py use_ekf:=True
ros2 topic echo /isaacbot/odom
```

## Known limitations

- EKF parameters are not visible in Foxglove's Parameters panel. robot_localization declares
  `odom1`, `imu1`, `pose0` and `twist0` without values while discovering sensors, so the
  bridge's bulk parameter request is rejected as a whole. The resulting rclcpp warning
  (`Failed to get parameters: parameter 'imu1' is not initialized`) is silenced in the launch
  file with `--log-level rclcpp:=ERROR`.
- Only the `rel` configs exist. `localization_mode:=enu` selects files that are not in the
  package, so pass `localization_config_path` with it.
- `isaacbot_amr_navsat_transform_node` computes its transform once, from the first heading it
  receives. After `gps/reset_heading_alignment` (isaacbot_amr_gps_heading), restart the node.
