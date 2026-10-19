# Copyright 2025 IsaacBot
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

"""Wheel geometry in the controller config must match isaacbot_amr_description.

isaacbot_amr_description/config/<wheel_type>.yaml is the single source of truth for
wheel geometry (it drives the URDF). diff_drive_controller needs the same
values as plain parameters, so this test fails the build if they drift.
"""

import math
from pathlib import Path
import re

from ament_index_python.packages import get_package_share_directory
import pytest
import yaml

CONTROLLER_CONFIG_DIR = Path(__file__).resolve().parent.parent / 'config'
WHEEL_TYPES = ['wheel_01']


def _load(path):
    with open(path, encoding='utf-8') as f:
        return yaml.safe_load(f)


@pytest.mark.parametrize('wheel_type', WHEEL_TYPES)
def test_drive_controller_matches_description(wheel_type):
    description_dir = Path(get_package_share_directory('isaacbot_amr_description')) / 'config'
    wheel = _load(description_dir / f'{wheel_type}.yaml')
    controller = _load(CONTROLLER_CONFIG_DIR / f'{wheel_type}_controller.yaml')

    drive = controller['/**']['isaacbot_amr_drive_controller']['ros__parameters']
    assert drive['wheel_radius'] == pytest.approx(wheel['wheel_radius'])
    assert drive['wheel_separation'] == pytest.approx(wheel['wheel_separation'])


@pytest.mark.parametrize('wheel_type', WHEEL_TYPES)
def test_drive_controller_drives_the_wheel_joints_directly(wheel_type):
    """The DCC1020s close the speed loop, so diff_drive must not chain through host wheel PIDs."""
    config = _load(CONTROLLER_CONFIG_DIR / f'{wheel_type}_controller.yaml')['/**']
    manager = config['controller_manager']['ros__parameters']
    drive = config['isaacbot_amr_drive_controller']['ros__parameters']
    wheels = drive['left_wheel_names'] + drive['right_wheel_names']
    assert len(wheels) == 2 * drive['wheels_per_side']
    for wheel in wheels:
        assert '/' not in wheel, wheel
        assert wheel.endswith('_wheel_joint'), wheel
    assert not [name for name, value in manager.items()
                if isinstance(value, dict) and
                value.get('type') == 'isaacbot_amr_controller/SeededPidController']


@pytest.mark.parametrize('wheel_type', WHEEL_TYPES)
def test_calibration_multipliers_are_plausible(wheel_type):
    """Skid-steer separation multipliers are > 1; radius multipliers stay near 1."""
    drive = _load(CONTROLLER_CONFIG_DIR / f'{wheel_type}_controller.yaml')[
        '/**']['isaacbot_amr_drive_controller']['ros__parameters']
    assert 1.0 <= drive['wheel_separation_multiplier'] <= 2.5
    for side in ('left', 'right'):
        assert 0.9 <= drive[f'{side}_wheel_radius_multiplier'] <= 1.1


def _urdf_wheel_velocity_limit():
    xacro = (Path(get_package_share_directory('isaacbot_amr_description'))
             / 'urdf' / 'common' / 'wheel.urdf.xacro').read_text(encoding='utf-8')
    match = re.search(r'<limit\b[^>]*\bvelocity="([0-9.]+)"', xacro)
    assert match, 'wheel joint <limit velocity="..."> not found in wheel.urdf.xacro'
    return float(match.group(1))


def _urdf_hardware_param(name):
    xacro = (Path(get_package_share_directory('isaacbot_amr_description'))
             / 'urdf' / 'ardennes' / 'ardennes_macro.urdf.xacro').read_text(encoding='utf-8')
    match = re.search(rf'<param name="{name}">([0-9.]+)</param>', xacro)
    assert match, f'<param name="{name}"> not found in ardennes_macro.urdf.xacro'
    return float(match.group(1))


@pytest.mark.parametrize('wheel_type', WHEEL_TYPES)
def test_drive_limits_respect_joint_velocity_limit(wheel_type):
    """Full linear + full angular must keep the outer wheel under the URDF limit.

    diff_drive limits linear.x and angular.z independently, and the outer wheel's reference is
    sent to its DCC1020 as-is, so it must also fit the motor-RPM clamp the driver applies
    (max_rpm_motor_speed / gear_ratio) - beyond it the wheel would silently run slower than
    diff_drive's kinematics assume.
    """
    config = _load(CONTROLLER_CONFIG_DIR / f'{wheel_type}_controller.yaml')['/**']
    drive = config['isaacbot_amr_drive_controller']['ros__parameters']
    half_track = drive['wheel_separation'] * drive['wheel_separation_multiplier'] / 2.0
    max_v = max(drive['linear']['x']['max_velocity'], -drive['linear']['x']['min_velocity'])
    max_w = max(drive['angular']['z']['max_velocity'], -drive['angular']['z']['min_velocity'])
    wheel_reference = (max_v + max_w * half_track) / drive['wheel_radius']

    limit = _urdf_wheel_velocity_limit()
    assert wheel_reference <= limit, (
        f'outer wheel reference {wheel_reference:.2f} rad/s exceeds the URDF limit {limit} rad/s')

    target_clamp = (_urdf_hardware_param('max_rpm_motor_speed') * 2.0 * math.pi / 60.0
                    / _urdf_hardware_param('gear_ratio'))
    assert wheel_reference <= target_clamp, (
        f'outer wheel reference {wheel_reference:.2f} rad/s exceeds the motor target clamp '
        f'{target_clamp:.2f} rad/s')


@pytest.mark.parametrize('wheel_type', WHEEL_TYPES)
def test_controller_update_rates_divide_manager_rate(wheel_type):
    """A per-controller update_rate must divide the controller_manager rate evenly.

    Otherwise controller_manager still runs it, but at an uneven period, and warns once.
    """
    config = _load(CONTROLLER_CONFIG_DIR / f'{wheel_type}_controller.yaml')['/**']
    manager = config['controller_manager']['ros__parameters']
    manager_rate = manager['update_rate']
    controllers = [name for name, value in manager.items()
                   if isinstance(value, dict) and 'type' in value]
    for name in controllers:
        rate = config.get(name, {}).get('ros__parameters', {}).get('update_rate')
        if rate is None:
            continue
        assert 0 < rate <= manager_rate, f'{name}: update_rate {rate} Hz'
        assert manager_rate % rate == 0, (
            f'{name}: update_rate {rate} Hz does not divide {manager_rate} Hz')
