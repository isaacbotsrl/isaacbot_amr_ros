# isaacbot_amr_bringup

Top-level launch files that start the whole Ardennes stack on real hardware. For simulation use
`isaacbot_amr_gazebo` instead.

## Launch Files

### `isaacbot_amr_bringup.launch.py`

Starts the isaacbot driver stack on hardware. Run it inside the `rover-a1-platform` container (`rover_docker`), or
directly:

```bash
ros2 launch isaacbot_amr_bringup isaacbot_amr_bringup.launch.py
```

It prints the isaacbot banner (`isaacbot_amr_utils.messages.welcome_msg`) and then:

1. Checks `ROBOT_HW_CONFIG_CORRECT` (default `true`). If it is not `true`, it logs
   `INCORRECT_HW_CONFIG` and starts nothing. It keeps the launch alive unless
   `exit_on_wrong_hw:=true`.
2. Warns when `SYSTEM_BUILD_VERSION` is older than the minimum OS version (`v1.0.0`).
3. Starts immediately:
   - `isaacbot_amr_controller` (robot description, `controller_manager`, controllers)
   - `isaacbot_amr_diag_manager` (system diagnostics and the diagnostic aggregator)
4. Once `isaacbot_amr_controller` emits `isaacbot_amr_utils.events.ControllersActive` (its last mandatory
   spawner, `isaacbot_imu_broadcaster`, succeeded), starts:
   - `isaacbot_amr_battery`
   - `isaacbot_amr_led`
   - `isaacbot_amr_safety` (skipped with `disable_manager:=True`)
   - `isaacbot_amr_localization` (with `use_ekf:=True` and `fuse_gps:=<use_gps>`)
   - `isaacbot_amr_crsf_teleop`
   - `isaacbot_amr_twist_mux`

   `isaacbot_amr_utils.events.start_once_on` starts them exactly once: on the event, or after
   `controllers_ready_timeout` seconds with a warning if it never comes. A failed spawner shuts
   the whole launch down, so the timeout only covers a spawner that hangs.

The GPS and lidar drivers are not started here. They are the sensor payload: the
`rover_sensors` repo, run by the `rover-a1-sensors` container, which only publishes `gps/fix`,
`scan`, `rslidar_points` and their diagnostics. In GPS mode `isaacbot_amr_localization` also starts
`isaacbot_gps_heading_node`, which turns `gps/fix` + `odom` into the ENU heading.

Every included launch file receives `namespace`, `log_level` and, where supported,
`common_dir_path`.

| Argument | Default | Description |
|----------|---------|-------------|
| `namespace` | `$ISAACBOT_SYSTEM_NAMESPACE`, else empty | Namespace of every node (the isaacbot uses `isaacbot`). |
| `log_level` | `INFO` | Logging level passed to every package. |
| `common_dir_path` | empty | Directory with per-package config overrides (`<dir>/<package>/config/...`). |
| `disable_manager` | `False` | `True` skips `isaacbot_amr_safety`. |
| `exit_on_wrong_hw` | `false` | Exit instead of idling when the hardware configuration is incorrect. |
| `controllers_ready_timeout` | `20.0` | Seconds to wait for `ControllersActive` before starting step 4 anyway. |
| `use_gps` | `$ISAACBOT_SYSTEM_USE_GPS`, else `false` | `true`: localization fuses wheels + IMU + GPS (dual EKF, `map → odom`). `false`: wheels + IMU only. |

| Environment variable | Default | Effect |
|----------------------|---------|--------|
| `ISAACBOT_SYSTEM_NAMESPACE` | empty | Default for `namespace`. |
| `ISAACBOT_SYSTEM_USE_GPS` | `false` | Default for `use_gps` (set as a balenaCloud variable; `start.sh` normalizes it to `true`/`false`). |
| `ROBOT_MODEL_NAME` / `ROBOT_SERIAL_NO` / `ROBOT_VERSION` | `ardennes` / `A1-2026-01` / `1.0` | Shown in the banner. |
| `ROBOT_HW_CONFIG_CORRECT` | `true` | Gate for starting the driver stack. |
| `SYSTEM_BUILD_VERSION` | `v1.0.0` | OS version compared against the minimum. |

### `isaacbot_web_bridges.launch.py`

Starts the web bridges under isaacbot-prefixed node names. `rover_docker/ardennes_platform/start.sh`
launches it next to the bringup.

| Node | Package | Notes |
|------|---------|-------|
| `isaacbot_foxglove_bridge` | `foxglove_bridge` | Upstream `foxglove_bridge_launch.xml` defaults except `asset_uri_allowlist`, `topic_whitelist` and `sysinfo` (below), websocket port 8765. Used by Foxglove, the network monitor LED page and the drive UI diagnostics. |
| `isaacbot_amr_rosbridge_websocket` | `rosbridge_server` | Port 9090, for ros-mcp-server. |
| `rosapi` | `rosapi` | Kept as `/rosapi`: rosbridge clients call `/rosapi/*`. |

The bridges are not namespaced, so they see the whole graph (`/isaacbot/...` topics included).

`isaacbot_foxglove_bridge` advertises only the topics the drive UI and other UI clients subscribe to
(`FOXGLOVE_TOPIC_WHITELIST` in the launch file: `/tf`, `/tf_static`, and names matched under any
namespace), because anything a browser subscribes to crosses the Zenoh router at full rate. When a
UI starts using a new topic, add it there. `ISAACBOT_PLATFORM_FOXGLOVE_TOPIC_WHITELIST="['.*']"` on the
platform service overrides it for debugging (empty counts as unset). `sysinfo:=false` turns off
`/foxglove_bridge/sysinfo`, which no UI reads and which otherwise publishes every 500 ms.

```bash
ros2 launch isaacbot_amr_bringup isaacbot_web_bridges.launch.py
```
