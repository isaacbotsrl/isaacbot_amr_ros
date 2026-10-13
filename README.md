<p align="center">
  <picture>
    <source media="(prefers-color-scheme: dark)" srcset="icons/isaacbot-lockup-white-bold.svg">
    <img src="icons/isaacbot-lockup-black-bold.svg" alt="IsaacBot" width="240">
  </picture>
</p>

# isaacbot_amr_ros

Mechatronics Academy's Ardennes ROS2.

**Bringup & simulation**

- [`isaacbot_amr_bringup`](isaacbot_amr_bringup/README.md) - top-level launch files that start the whole
  stack on real hardware.
- [`isaacbot_amr_gazebo`](isaacbot_amr_gazebo/README.md) - Gazebo Sim bringup: the robot with
  `gz_ros2_control`, the same controllers, EKFs and twist_mux as the real isaacbot, simulated
  lidar and GNSS, the ROS–Gazebo bridge and RViz.
- [`isaacbot_amr_world`](isaacbot_amr_world/README.md) - Gazebo Sim worlds for the simulation.
- [`isaacbot_amr_description`](isaacbot_amr_description/README.md) - URDF/xacro model: links, joints,
  meshes, sensors and the `<ros2_control>` hardware components.
- [`isaacbot_amr_metapackage`](isaacbot_amr_metapackage/README.md) - metapackage pulling in the entry point
  for the build type, plus the `vcs` `.repos` dependency lists.

**Control & hardware**

- [`isaacbot_amr_hardware_interface`](isaacbot_amr_hardware_interface/README.md) - ros2_control plugins
  `ArdennesSystem` (wheel motors and safety controller) and `PhidgetImuSensor`.
- [`isaacbot_amr_controller`](isaacbot_amr_controller/README.md) - ros2_control controller configuration
  and launch.
- [`isaacbot_amr_twist_mux`](isaacbot_amr_twist_mux/README.md) - velocity command arbitration with
  `twist_mux`, plus `isaacbot_amr_motion_lock_node`.
- [`isaacbot_amr_crsf_teleop`](isaacbot_amr_crsf_teleop/README.md) - CRSF (ExpressLRS) RC teleop, software
  E-Stop switches and RC link failsafe.

**Localization**

- [`isaacbot_amr_localization`](isaacbot_amr_localization/README.md) - `robot_localization` EKFs fusing
  wheel odometry and IMU, optionally GPS (`ISAACBOT_AMR_SYSTEM_USE_GPS`).
- [`isaacbot_amr_gps_heading`](isaacbot_amr_gps_heading/README.md) - ENU heading from GNSS course for
  `navsat_transform_node`.

**Safety, power & status**

- [`isaacbot_amr_safety`](isaacbot_amr_safety/README.md) - behavior-tree safety supervision: software
  E-Stop, shutdown and LED animation selection.
- [`isaacbot_amr_battery`](isaacbot_amr_battery/README.md) - decodes BMS telemetry received over UDP and
  publishes the battery state.
- [`isaacbot_amr_led`](isaacbot_amr_led/README.md) - SK9822 bumper LED panels: animation controller and
  driver.
- [`isaacbot_amr_diag_manager`](isaacbot_amr_diag_manager/README.md) - the isaacbot's diagnostic nodes.

**Interfaces, transport & utilities**

- [`isaacbot_amr_msgs`](isaacbot_amr_msgs/README.md) - custom messages and services.
- [`isaacbot_amr_utils`](isaacbot_amr_utils/README.md) - shared header-only C++ utilities and Python launch
  helpers.
- [`isaacbot_amr_transport`](isaacbot_amr_transport/README.md) - hard fork of `transport_drivers`: serial,
  UDP and Modbus TCP drivers on top of `isaacbot_amr_io_context`.
- [`isaacbot_amr_modbus`](isaacbot_amr_modbus/README.md) - vendored Modbus library for modern C++.

**Not ROS packages**

- [`isaacbot_amr_arch`](isaacbot_amr_arch/README.md) - firmware architecture docs and diagrams, including
  the safety chain.
- [`isaacbot_amr_foxglove`](isaacbot_amr_foxglove/README.md) - Foxglove dashboard layout for the `isaacbot`
  namespace.
- [`isaacbot_amr_scripts`](isaacbot_amr_scripts/README.md) - development PC helper scripts
  (`setup_isaacbot_amr_pc.sh`).
- [`rover_platform_mbse`](rover_platform_mbse/README.md) - MATLAB/Simulink MBSE project for
  the platform: System Composer architecture, per-package software requirements
  (Requirements Toolbox), behaviour models and tests, traceability and SYS-SR compliance.
- [`docs`](docs/index.md) - the platform manual (MkDocs), published at
  <https://isaacbotsrl.github.io/isaacbot_amr_ros/>. See [Platform manual](#platform-manual).

## Quick start

### Create workspace

```bash
mkdir -p ~/ros2_ws/rover_a1
cd ~/ros2_ws/rover_a1
git clone -b master https://github.com/isaacbotsrl/isaacbot_amr_ros.git src/isaacbot_amr_ros
```

### Setup environment variables

One step: write the **isaacbot-pc setup** block into `~/.bashrc`. It covers ROS, the workspace
overlay, `ISAACBOT_AMR_*`, and the Zenoh settings to reach the isaacbot at `192.168.1.201`. It's safe to
re-run; see [isaacbot_amr_scripts/README.md](isaacbot_amr_scripts/README.md) for the options.

```bash
src/isaacbot_amr_ros/isaacbot_amr_scripts/setup_isaacbot_amr_pc.sh
source ~/.bashrc
```

Or by hand:

```bash
export ROS_DISTRO=lyrical
export ISAACBOT_AMR_SYSTEM_NAMESPACE=isaacbot
```

#### Real isaacbot:

```bash
export ISAACBOT_AMR_ROS_BUILD_TYPE=hardware
```

#### Simulated isaacbot:

```bash
export ISAACBOT_AMR_ROS_BUILD_TYPE=simulation
```

### Clone dependency

```bash
vcs import src < src/isaacbot_amr_ros/isaacbot_amr_metapackage/${ISAACBOT_AMR_ROS_BUILD_TYPE}_deps.repos
```

### Build

```bash
sudo apt install usbutils
sudo apt install plocate

sudo rosdep init
rosdep update --rosdistro $ROS_DISTRO
rosdep install --from-paths src -y -i
```

#### Only for real isaacbot

```bash
cd src/isaacbot_amr_cppuprofile/
cmake -Bbuild . -DPROFILE_ENABLED=ON
cmake --build build
cd build
sudo make install
cd ../../..
```

#### For real isaacbot and simulated isaacbot

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --symlink-install --packages-up-to isaacbot_amr_metapackage --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF

source install/setup.bash
```

### Running

#### Real isaacbot:

```bash
ros2 launch isaacbot_amr_bringup isaacbot_amr_bringup.launch.py
```

#### Simulated isaacbot:

```bash
src/isaacbot_amr_ros/isaacbot_amr_gazebo/scripts/isaacbot_amr_sim.sh
```

### Testing

```bash
# The build above passes -DBUILD_TESTING=OFF, so rebuild the package under test with it on.
colcon build --symlink-install --packages-select <pkg>
colcon test --packages-select <pkg> --parallel-workers 1
colcon test-result --all
```

Run node tests one at a time: parallel workers make them flaky under the zenoh middleware.

## Platform manual

The platform manual is a [MkDocs Material](https://squidfunk.github.io/mkdocs-material/) site
built from `docs/` and `mkdocs.yml`. It's published at
<https://isaacbotsrl.github.io/isaacbot_amr_ros/>. Run the commands below from the repository root
(`src/isaacbot_amr_ros`).

### Preview locally

Serve the manual with live reload at <http://127.0.0.1:8000>:

```bash
uvx --with 'mkdocs-material>=9.5,<10' --with 'mkdocs>=1.6,<2' mkdocs serve
```

Or, without `uv`:

```bash
python3 -m venv .venv-docs
source .venv-docs/bin/activate
pip install -r docs/requirements.txt
mkdocs serve
```

### Build the static site

```bash
mkdocs build --strict
```

The output goes to `site/`, which is git-ignored. CI uses `--strict`, so broken links or a
page missing from `nav:` fail the build. Run it before you push.

### Regenerate the isaacbot renders

After a mesh or URDF change, re-render the images in `docs/assets/images/`
(`ardennes_iso.png`, `ardennes_top.png`, `ardennes_side.png`). The script also prints the
overall dimensions, so update the manual pages that quote them:

```bash
uv run --with trimesh,matplotlib,numpy,pyyaml,fast-simplification python docs/tools/render_meshes.py
```

### Publish

[`.github/workflows/docs.yml`](.github/workflows/docs.yml) builds the site and deploys it to
GitHub Pages on every push to `master` that touches `docs/**`, `mkdocs.yml` or the workflow.
To deploy without a push, run the **docs** workflow from the Actions tab.

One-time setup: GitHub → **Settings → Pages → Build and deployment → Source** = **GitHub
Actions**.

### Editing conventions

- Keep MkDocs below 2.0, because 2.0 breaks Material. The pins are in `docs/requirements.txt`.
- Add every new page to `nav:` in `mkdocs.yml`.
- Every value cites the repository file it comes from. If the repository doesn't define a
  value, write **TBD** and list it in `docs/open-items.md`. List source conflicts there too.

## Related repositories

A complete isaacbot is three repositories, one per container:

- [`isaacbot_amr_ros`](https://github.com/isaacbotsrl/isaacbot_amr_ros) - this one, the platform
  (`rover-a1-platform`).
- [`isaacbot_amr_sensors`](https://github.com/isaacbotsrl/isaacbot_amr_sensors) - the sensor payload,
  GNSS and lidar drivers (`rover-a1-sensors`).
- [`isaacbot_amr_orchestrator`](https://github.com/isaacbotsrl/isaacbot_amr_orchestrator) - the autonomy
  stack, Nav 2 and mission supervision (`rover-a1-orchestrator`).
