#!/usr/bin/env python3

# Copyright 2026 IsaacBot
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Step the body velocity, record each wheel's commanded and measured speed, report the response.

    ros2 run isaacbot_amr_controller wheel_step_response --ros-args -r __ns:=/<ns> \\
        -p enable_motion:=true

The reference is the step itself, turned into each wheel's speed with the drive controller's
kinematics; the feedback is the wheel velocity in joint_states (the DCC1020's OnVelocityChange).
So the response covers the whole chain: diff_drive's limiter, then each board's ramp and PID.

Writes <output_dir>/samples.csv and summary.yaml and logs recommended acceleration limits.
"""

import csv
import os
from statistics import median

import rclpy
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import JointState
import yaml

from isaacbot_amr_controller.drive_session import DriveSession
from isaacbot_amr_controller.schedule import active_segment, build_schedule
from isaacbot_amr_controller.response_analysis import (
    analyze_step, derive_acceleration_limits, finite, Sample)

# Must match isaacbot_amr_drive_controller's left_wheel_names / right_wheel_names.
LEFT_WHEELS = ['rl_wheel_base_to_rl_wheel_joint', 'fl_wheel_base_to_fl_wheel_joint']
RIGHT_WHEELS = ['rr_wheel_base_to_rr_wheel_joint', 'fr_wheel_base_to_fr_wheel_joint']


class WheelStepResponse(DriveSession):

    def __init__(self):
        super().__init__('wheel_step_response')
        self.declare_parameter('left_wheels', LEFT_WHEELS)
        self.declare_parameter('right_wheels', RIGHT_WHEELS)
        self.declare_parameter('linear_steps', [0.2, 0.5, 1.0, 1.2, -0.5])
        self.declare_parameter('angular_steps', [0.5, 1.0, 1.5, -1.0])
        # Must match isaacbot_amr_drive_controller: the wheel reference and the body limits use it.
        self.declare_parameter('wheel_radius', 0.1651)
        self.declare_parameter('wheel_separation', 0.617)
        self.declare_parameter('wheel_separation_multiplier', 1.659)
        self.declare_parameter('margin', 0.8)
        self._left = set(self.get_parameter('left_wheels').value)
        self._right = set(self.get_parameter('right_wheels').value)
        self.samples = {}  # wheel -> list of (t, reference, feedback)
        self.create_subscription(JointState, 'joint_states', self._on_joints, 50)

    def _wheel_reference(self, wheel, t):
        segment = active_segment(self.schedule, t)
        if segment is None:
            return 0.0
        half_track = (self.get_parameter('wheel_separation').value *
                      self.get_parameter('wheel_separation_multiplier').value / 2.0)
        side = -1.0 if wheel in self._left else 1.0
        return ((segment.linear + side * segment.angular * half_track) /
                self.get_parameter('wheel_radius').value)

    def _on_joints(self, msg):
        if self._start is None:
            return
        t = self.elapsed()
        for name, velocity in zip(msg.name, msg.velocity):
            if name in self._left or name in self._right:
                self.samples.setdefault(name, []).append(
                    (t, self._wheel_reference(name, t), velocity))

    def start(self) -> bool:
        commands = [(v, 0.0, f'linear {v:+.2f}')
                    for v in self.get_parameter('linear_steps').value]
        commands += [(0.0, w, f'angular {w:+.2f}')
                     for w in self.get_parameter('angular_steps').value]
        return self.run(build_schedule(commands, self.get_parameter('hold_time').value,
                                       self.get_parameter('rest_time').value))

    def report(self):
        if not self.samples:
            self.get_logger().error(
                'No wheel velocities in joint_states - is the joint state broadcaster active in '
                'this namespace?')
            return
        out = self.output_dir()
        with open(os.path.join(out, 'samples.csv'), 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['wheel', 't', 'reference', 'feedback'])
            for wheel, rows in self.samples.items():
                writer.writerows([wheel, *row] for row in rows)

        rest = self.get_parameter('rest_time').value
        steps = []
        for segment in self.schedule:
            if segment.label == 'rest':
                continue
            per_wheel = {}
            for wheel, rows in self.samples.items():
                window = [Sample(t, ref, fb) for t, ref, fb in rows
                          if segment.start - rest / 2 <= t < segment.end]
                try:
                    per_wheel[wheel] = analyze_step(window, segment.start)
                except ValueError:
                    continue
            if not per_wheel:
                continue
            metric = list(per_wheel.values())

            def med(values):
                values = [v for v in values if finite(v)]
                return round(median(values), 4) if values else None

            steps.append({
                'step': segment.label,
                'wheel_accel_rad_s2': med([m.max_acceleration for m in metric]),
                'dead_time_s': med([m.dead_time for m in metric]),
                'rise_time_s': med([m.rise_time for m in metric]),
                'overshoot': med([m.overshoot for m in metric]),
                'steady_state_error': med([m.steady_state_error for m in metric]),
            })
            self.get_logger().info(f'{steps[-1]}')

        accels = [s['wheel_accel_rad_s2'] for s in steps if s['wheel_accel_rad_s2']]
        summary = {'steps': steps}
        if accels:
            # Conservative: the weakest step bounds what every step can rely on.
            alpha = min(accels)
            limits = derive_acceleration_limits(
                alpha, self.get_parameter('wheel_radius').value,
                self.get_parameter('wheel_separation').value,
                self.get_parameter('wheel_separation_multiplier').value,
                self.get_parameter('margin').value)
            summary['wheel_accel_rad_s2'] = alpha
            summary['recommended'] = {
                'linear_accel_m_s2': round(limits.linear, 3),
                'angular_accel_rad_s2': round(limits.angular, 3),
            }
            self.get_logger().info(
                f'Wheel accel {alpha:.2f} rad/s^2 -> linear.x {limits.linear:.2f} m/s^2, '
                f'angular.z {limits.angular:.2f} rad/s^2 (margin '
                f'{self.get_parameter("margin").value}). If these sit at the drive controller '
                'limits, diff_drive (or the board ramp, motor_acceleration_rpm_s) was the '
                'bottleneck - relax it and re-measure.')
        with open(os.path.join(out, 'summary.yaml'), 'w') as f:
            yaml.safe_dump(summary, f, sort_keys=False)
        self.get_logger().info(f'Wrote {out}')


def main(args=None):
    rclpy.init(args=args)
    node = WheelStepResponse()
    try:
        if node.start():
            while rclpy.ok() and not node.finished:
                rclpy.spin_once(node, timeout_sec=0.1)
            node.report()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.stop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
