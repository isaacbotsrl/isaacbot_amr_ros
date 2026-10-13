# Copyright 2026 Mechatronics Academy
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

"""foxglove_bridge's topic and service whitelists cover what the web UIs use, and nothing else."""

import importlib.util
from pathlib import Path
import re

import pytest
import yaml


@pytest.fixture(scope='module')
def launch_module():
    path = Path(__file__).resolve().parents[1] / 'launch' / 'isaacbot_web_bridges.launch.py'
    spec = importlib.util.spec_from_file_location('isaacbot_web_bridges_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope='module')
def patterns(launch_module):
    return yaml.safe_load(launch_module.FOXGLOVE_TOPIC_WHITELIST)


@pytest.fixture(scope='module')
def service_patterns(launch_module):
    return yaml.safe_load(launch_module.FOXGLOVE_SERVICE_WHITELIST)


def _allowed(patterns, topic):
    # foxglove_bridge matches the whole topic name (std::regex_match).
    return any(re.fullmatch(p, topic) for p in patterns)


# Published by rover_drive_interface. The UI's foxglove client takes a publisher's message
# definition from the server channel with the same name, so a publish-only topic missing here
# makes it wait forever and send nothing: manual driving was silently dead on 2026-09-26.
@pytest.mark.parametrize('topic', [
    '/isaacbot/teleop_web_cmd_vel_stamped',
    '/isaacbot/initialpose',
])
def test_topics_the_drive_ui_publishes_are_advertised(patterns, topic):
    assert _allowed(patterns, topic)


@pytest.mark.parametrize('topic', [
    '/tf', '/tf_static', '/isaacbot/map', '/isaacbot/scan', '/isaacbot/plan',
    '/isaacbot/global_costmap/costmap', '/isaacbot/diagnostics_agg', '/isaacbot/motion_lock',
    '/isaacbot/hardware_interface/safety_status', '/isaacbot/battery/battery_status',
    '/isaacbot/mission_state', '/isaacbot/drive_mode',
    '/isaacbot/led/channel_1_preview', '/isaacbot/rc/channels', '/isaacbot/rc/calibration/state',
    '/isaacbot/follow_me/status',
])
def test_topics_the_uis_subscribe_to_are_advertised(patterns, topic):
    assert _allowed(patterns, topic)


def test_namespace_is_not_hard_coded(patterns):
    assert _allowed(patterns, '/isaacbot_a2/scan')
    assert _allowed(patterns, '/scan')


@pytest.mark.parametrize('topic', [
    # Full-rate or internal topics a browser must not pull through the router.
    '/isaacbot/led/channel_1_frame', '/isaacbot/rslidar_points', '/isaacbot/diagnostics', '/rosout',
    '/isaacbot/cmd_vel', '/isaacbot/teleop_driver_interface_cmd_vel_fresh_stamped',
    # rover_drive_mode's output: a browser publishing here would bypass the driving mode.
    '/isaacbot/teleop_driver_interface_cmd_vel_stamped', '/isaacbot/nav_cmd_vel_stamped',
    '/isaacbot/odometry/wheels', '/isaacbot/joint_states',
])
def test_internal_topics_stay_off_the_bridge(patterns, topic):
    assert not _allowed(patterns, topic)


@pytest.mark.parametrize('service', [
    '/rosapi/get_time',
    '/isaacbot/hardware_interface/sw_user_e_stop_set',
    '/isaacbot/hardware_interface/sw_user_e_stop_reset',
    '/isaacbot/hardware_interface/sw_e_stop_latch_reset',
    '/isaacbot/hardware_interface/aux_output_0/set',
    '/isaacbot/hardware_interface/aux_output_5/set',
    '/isaacbot/set_drive_mode', '/isaacbot/set_mission', '/isaacbot/run_mission',
    '/isaacbot/start_mapping', '/isaacbot/save_map', '/isaacbot/load_map', '/isaacbot/delete_map',
    '/isaacbot/save_place', '/isaacbot/delete_place',
    '/isaacbot/reinitialize_global_localization', '/isaacbot/request_nomotion_update',
    '/isaacbot/rc/calibration/start', '/isaacbot/rc/calibration/apply',
    '/isaacbot/isaacbot_crsf_teleop_node/change_state', '/isaacbot/isaacbot_crsf_teleop_node/get_state',
    '/isaacbot/led/set_animation', '/isaacbot/led/stop_animation', '/isaacbot/led/set_brightness',
    '/isaacbot/follow_me/start', '/isaacbot/follow_me/stop',
    # Unnamespaced isaacbot.
    '/save_map', '/led/set_brightness',
])
def test_services_the_uis_call_are_advertised(service_patterns, service):
    assert _allowed(service_patterns, service)


@pytest.mark.parametrize('service', [
    # Node-private services sharing a leaf name with a UI service: their packages aren't in the
    # platform image, so advertising them only made the bridge log "package not found".
    '/isaacbot/slam_toolbox/save_map', '/isaacbot/map_saver/save_map',
    '/isaacbot/global_costmap/save_grid',
    # Everything else on the graph.
    '/isaacbot/controller_server/get_parameters', '/isaacbot/isaacbot_led_driver/change_state',
    '/rosapi/topics',
])
def test_other_services_stay_off_the_bridge(service_patterns, service):
    assert not _allowed(service_patterns, service)
