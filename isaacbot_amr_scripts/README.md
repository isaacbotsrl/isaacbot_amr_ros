# isaacbot_amr_scripts

Helper scripts for a development PC working with Ardennes. This folder has no `package.xml`, so
colcon ignores it.

## `setup_isaacbot_amr_pc.sh`

Writes the **isaacbot-pc setup** block into `~/.bashrc`. The block sets up:
- ROS 2 (`/opt/ros/<distro>/setup.bash`);
- the ardennes workspace overlay (sourced only once it is built);
- `ISAACBOT_AMR_ROS_BUILD_TYPE` and `ISAACBOT_AMR_SYSTEM_NAMESPACE`;
- the Zenoh settings that put this PC on the isaacbot's ROS graph;
- `~/.local/bin` and `~/balena/bin` on PATH, and `QT_QPA_PLATFORM=xcb`.

```bash
~/ros2_ws/ardennes/src/isaacbot_amr_ros/isaacbot_amr_scripts/setup_isaacbot_amr_pc.sh              # defaults below
~/ros2_ws/ardennes/src/isaacbot_amr_ros/isaacbot_amr_scripts/setup_isaacbot_amr_pc.sh --isaacbot-ip 10.0.0.5 --dry-run
~/ros2_ws/ardennes/src/isaacbot_amr_ros/isaacbot_amr_scripts/setup_isaacbot_amr_pc.sh --remove
source ~/.bashrc   # or open a new terminal
```

| Option | Default |
|--------|---------|
| `--workspace PATH` | the workspace this script lives in (`<ws>/src/isaacbot_amr_ros/isaacbot_amr_scripts`) |
| `--isaacbot-ip IP` | `192.168.1.201` |
| `--namespace NAME` | `isaacbot` |
| `--distro NAME` | `lyrical` (or the only distro under `/opt/ros`) |
| `--build-type hardware\|simulation` | `hardware` |
| `--domain-id N` | `0` (as on the isaacbot) |
| `--bashrc FILE` | `~/.bashrc` |
| `--dry-run` | print the block and the diff, change nothing |
| `--remove` | delete the block |

- **Idempotent.** The block between `# >>> isaacbot-pc setup >>>` and `# <<< isaacbot-pc setup <<<` is
  replaced in place, and the rest of the file is left byte for byte. If nothing changed, nothing
  is written.
- **Backups.** A backup (`~/.bashrc.bak-YYYYmmdd-HHMMSS`) is made before every change.
- **Broken markers.** Unbalanced or duplicated markers are refused rather than guessed at.
- **Edits.** Don't edit inside the block; re-run the script with other options instead.

### How the PC joins the isaacbot's graph

Every ROS node on the PC is a **direct Zenoh client** of the router in `ardennes-zenoh-router`, via
`ZENOH_CONFIG_OVERRIDE='mode="client";connect/endpoints=["tcp/<isaacbot-ip>:7447"]'`. No local
router runs. This is deliberate:
- **The simulation stays isolated.** `isaacbot_amr_gazebo/scripts/isaacbot_amr_sim.sh` clears the override and
  starts its own router on `127.0.0.1:7447`. A permanent local router that dialed the isaacbot would
  sit on that same port, be reused by `isaacbot_amr_sim.sh`, and bridge the simulated `/isaacbot` into the
  real isaacbot's `/isaacbot` graph.
- **The trade-off:** with the isaacbot off or unreachable, nodes started from a plain shell cannot
  connect. For the simulation use `isaacbot_amr_sim.sh`; for off-isaacbot `colcon test`, clear
  `ZENOH_CONFIG_OVERRIDE` and run a local router.
