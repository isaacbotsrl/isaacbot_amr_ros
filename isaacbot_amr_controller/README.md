# isaacbot_amr_controller

Configuration and launch composition for the Ardennes ros2_control controllers.
Controller lifecycle and hardware interfaces are managed by controller_manager.

## Configuration

`config/wheel_01_controller.yaml` configures the 4WD Isaacbot with 13-inch-diameter
wheels. Physical wheel geometry matches `isaacbot_amr_description`; calibration
multipliers are separate tuning parameters.

The wheel-speed loop runs on the motor controllers, not in ros2_control:

```
cmd_vel -> isaacbot_amr_drive_controller (diff_drive) -> hardware (4x DCC1020, on-board speed loop)
```

`isaacbot_amr_drive_controller`'s wheel names are the plain joint names, so it writes the
wheel joints' `velocity` command interfaces directly and reads their measured velocity for
odometry. Each Phidget DCC1020 closes the loop on board (`MotorVelocityController`:
trapezoidal profile + PID, in motor RPM); its gains are URDF parameters in
`isaacbot_amr_description/urdf/ardennes/ardennes_macro.urdf.xacro` (see
`isaacbot_amr_hardware_interface/README.md`). `publish_limited_velocity` is `false`: `cmd_vel_out`
was only read by the old PIDs' turn feed-forward. `test/test_wheel_geometry.py` checks that
diff_drive drives the joints directly and that no `SeededPidController` is configured.

### SeededPidController (built, not spawned)

Until the DCC1020 swap the motor controllers (DCC1000) only took a duty cycle and reported the
encoders at 10 Hz, so the host closed the loop with four `isaacbot_amr_controller/SeededPidController`s
chained under diff_drive (wheel names `<pid controller>/<joint>`). The controller is still built
and tested, but no config spawns it. It is `pid_controller/PidController` with its exported state
seeded on activation (so a chained diff_drive never reads NaN on its first cycle) and each
wheel's command computed by `WheelSpeedLoop` (`stop_at_zero_reference`, a model-reference
integral, `scale_integral_with_reference`, turn feed-forward). With every option off,
`WheelSpeedLoop` gives exactly `control_toolbox::Pid`'s output (`test/test_wheel_speed_loop.cpp`).
Its tuning history is in `docs/wheel_pid_tuning_notes.md` and in this README's git history.

The joint-state and IMU broadcasters are spawned as before.

controller_manager runs at 25 Hz, and each controller at its own `update_rate`:
`isaacbot_amr_drive_controller`, `isaacbot_amr_imu_broadcaster` and
`isaacbot_amr_joint_state_broadcaster` all at the manager's 25 Hz (the EKF runs at 50 Hz).
Every rate must divide the manager's (`test/test_wheel_geometry.py`). The manager used to run
at 100 Hz, then at 50 Hz, where 22-24 % of the velocity commands were dropped because the
DCC1000's asynchronous call took ~17 ms (median) to complete. The DCC1020 has no asynchronous
setter: each wheel's command worker sends the newest target with a blocking call and coalesces
newer ones (`dropped_pending` in the `command path` diagnostic). Re-measure that latency before
raising the rate (see the comment above the rates in `config/wheel_01_controller.yaml`).

## Drive-train tuning

Do these in order; each step relies on the one before. Both tools publish on the
twist_mux Foxglove input (priority 100, still gated by the E-Stop motion lock),
print their plan and only move the isaacbot with `-p enable_motion:=true`. Results go
to `~/isaacbot_amr_calibration/<tool>_<timestamp>/` (`summary.yaml`, raw samples).

1. **Wheel-speed loop.** Tune the DCC1020 on-board loop (`velocity_kp/ki/kd`,
   `velocity_deadband_rpm`, `motor_acceleration_rpm_s` in the URDF; all TBD, board defaults
   until set). At configure the driver logs the values each board runs with ("Motor hub port N
   (serial S): Kp=... (board default) ..."); start from those. Run the step-response tool with
   the wheels off the ground first, then on the ground, until overshoot stays under 10 % and
   steady-state error under 3 %. `docs/wheel_pid_tuning_notes.md` is the history of the old
   host PIDs on the DCC1000 and does not apply to the on-board loop.

   ```bash
   ros2 run isaacbot_amr_controller wheel_step_response --ros-args -r __ns:=/<ns> -p enable_motion:=true
   ```

2. **Acceleration limits.** The same run reports the largest wheel acceleration it
   saw and the body limits that follow from it (`margin` 0.8):
   `a = margin * r * alpha`, `alpha_z = 2 * margin * r * alpha / (wheel_separation * multiplier)`.
   If the result equals the current `linear.x/angular.z` limits in this file, the
   controller ramp was the bottleneck, not the wheels. Relax those limits for one
   measurement run and repeat. The current limits (1.4 m/s², 1.3 rad/s²) were
   measured with the DCC1000 and are TBD for the DCC1020; the on-board profile
   (`motor_acceleration_rpm_s`) adds lag of its own, so re-measure after changing it.
   Then set the limits from the inside out:
   `isaacbot_amr_drive_controller` >= Nav2 `velocity_smoother` `max_accel`/`max_decel`
   = MPPI `ax_max`/`ax_min`/`az_max` >= `behavior_server.rotational_acc_lim`.

3. **Skid-steer calibration.** Spin in place at several rates in both directions.
   The tool compares the rotation the measured wheel speeds explain with the IMU
   gyro and reports `wheel_separation_multiplier`. Run it on the surface the isaacbot
   mostly drives on, because the value depends on the surface. The IMU is mounted
   upside down, so the tool negates the gyro yaw rate (`imu_yaw_sign` -1); if every
   spin segment is rejected although the isaacbot turned, re-run with `-p imu_yaw_sign:=1`.

   ```bash
   ros2 run isaacbot_amr_controller wheel_odom_calibration --ros-args -r __ns:=/<ns> -p mode:=spin -p enable_motion:=true
   ros2 run isaacbot_amr_controller wheel_odom_calibration --ros-args -r __ns:=/<ns> -p mode:=straight -p distance:=5.0 -p enable_motion:=true
   ```

   `straight` stops when wheel odometry reads `distance`. Tape-measure the real
   distance `d`; then `left/right_wheel_radius_multiplier = d / distance`.
   `test/test_wheel_geometry.py` keeps both multipliers in plausible ranges.

## Launch

`isaacbot_amr_controller.launch.py` loads the robot description and starts the controllers.
On hardware it launches `ros2_control_node`; with `use_sim:=True`, the simulation's
`gz_ros2_control` plugin owns the controller manager.

Configuration selection, in descending precedence:

1. Explicit `controller_config_path`.
2. `<common_dir_path>/isaacbot_amr_controller/config/<wheel_type>_controller.yaml`.
3. The package's bundled `config/<wheel_type>_controller.yaml`.

The selected file is used by the manager, every spawner, and the simulation
robot description. `<namespace>/` placeholders in the file are replaced with
the requested namespace prefix (or an empty string for the root namespace).

Startup activates the joint-state broadcaster, drive controller, then IMU
broadcaster. The drive controller's spawner also takes every controller named as
`<controller>/<joint>` in its wheel names and activates them with it as one group
(`--activate-as-group`), so controller_manager orders a chain; the current config has
plain joint names, so it spawns no extra controllers. Each step requires the previous spawner to succeed. A mandatory
spawner failure reports its controller name and exit code and shuts down the
launch; subsequent controllers are not started.

## Topic remapping

Controller topics are remapped with each controller's `node_options_args` under
`controller_manager` in the config file, **not** with launch `remappings=` on
`ros2_control_node` or `<remapping>` tags in the Gazebo plugin. controller_manager
creates every controller as its own node with `use_global_arguments(false)`, so
process-wide remaps only affect the controller manager itself.

```yaml
controller_manager:
  ros__parameters:
    isaacbot_amr_drive_controller:
      type: diff_drive_controller/DiffDriveController
      node_options_args: ["-r", "~/odom:=odometry/wheels"]
```

`--ros-args` is prepended automatically; remap targets are relative, so they follow
the namespace. The spawner's `--controller-ros-args` sets the same parameter.
Resulting topics (in the isaacbot namespace): `cmd_vel` (drive controller input),
`odometry/wheels`, `imu/data`, `joint_states`; lifecycle `transition_event` topics
are hidden under `_<controller>/`.
