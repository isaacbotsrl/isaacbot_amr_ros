# isaacbot_amr_metapackage

Metapackage for the Ardennes ROS stack. Building it pulls in the right entry point for the
build type. The package also holds the `vcs` repository lists for the isaacbot's external
repositories.

## Dependencies

The dependencies are selected by `ISAACBOT_AMR_ROS_BUILD_TYPE`:

| `ISAACBOT_AMR_ROS_BUILD_TYPE` | Depends on |
|------------------------|------------|
| `hardware` | `isaacbot_amr_bringup` |
| `simulation` | `isaacbot_amr_gazebo` |

Its version (`ros2 pkg xml -t version isaacbot_amr_metapackage`) is shown as the "ROS Driver Version"
in the `isaacbot_amr_bringup` banner.

## Repository lists

| File | Repositories |
|------|--------------|
| `hardware_deps.repos` | `isaacbot_amr_modbus`, `rover_cppuprofile` (both `master`) |
| `simulation_deps.repos` | none (the file has a `repositories:` key with no entries) |

The transport packages (`isaacbot_amr_asio_cmake_module`, `isaacbot_amr_io_context`,
`isaacbot_amr_udp_driver`) are **vendored in-tree** under
`isaacbot_amr_transport/`, not pulled by `vcs`. They are a hard fork of
[ros-drivers/transport_drivers](https://github.com/ros-drivers/transport_drivers)
v1.2.0 - see `isaacbot_amr_transport/README.md`.

```bash
cd ~/ros2_ws/ardennes
export ISAACBOT_AMR_ROS_BUILD_TYPE=hardware   # or simulation
vcs import src < src/isaacbot_amr_ros/isaacbot_amr_metapackage/${ISAACBOT_AMR_ROS_BUILD_TYPE}_deps.repos
rosdep install --from-paths src -y -i
colcon build --symlink-install
```

`rover_docker/rovera1_app/Dockerfile` runs the same `vcs import` with `hardware_deps.repos`.
`isaacbot_amr_orchestrator` (Nav 2) is not listed, so clone it separately on the navigation computer.
