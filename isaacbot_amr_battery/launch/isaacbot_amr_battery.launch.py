#!/usr/bin/env python3

# Copyright 2025 Mechatronics Academy
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

from isaacbot_amr_utils.logging import limit_log_level_to_info
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    log_level = LaunchConfiguration("log_level")
    declare_log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="INFO",
        choices=["DEBUG", "INFO", "WARN", "ERROR", "FATAL"],
        description="Logging level",
    )

    namespace = LaunchConfiguration("namespace")
    declare_namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value=EnvironmentVariable("ISAACBOT_AMR_SYSTEM_NAMESPACE", default_value=""),
        description="Add namespace to all launched nodes",
    )

    isaacbot_amr_battery_config_path = LaunchConfiguration("isaacbot_amr_battery_config_path")
    declare_isaacbot_amr_battery_config_path_arg = DeclareLaunchArgument(
        "isaacbot_amr_battery_config_path",
        default_value=PathJoinSubstitution(
            [
                FindPackageShare("isaacbot_amr_battery"),
                "config",
                "isaacbot_amr_battery.yaml",
            ]
        ),
        description="Specify the path to the isaacbot battery configuration file.",
    )

    # The BMS UDP endpoint and the node that decodes its packets, in one container with
    # intra-process communication, so the packets never cross the Zenoh router. The receiver is a
    # lifecycle node brought to active by its own `autostart` parameter: launch_ros'
    # ComposableLifecycleNode autostart misses the namespace and never reaches it.
    isaacbot_amr_udp_battery_receiver_node = ComposableNode(
        package="isaacbot_amr_udp_driver",
        plugin="isaacbot::transport::udp::UdpReceiverNode",
        name="isaacbot_amr_udp_battery_receiver_node",
        namespace=namespace,
        parameters=[isaacbot_amr_battery_config_path, {"autostart": True}],
        # Topic names carry no isaacbot_amr_ prefix, so the lifecycle transition_event topic is remapped
        # off the node name (the node keeps it).
        remappings=[
            ("udp_read", "battery_udp_data"),
            ("~/transition_event", "udp_battery_receiver_node/transition_event"),
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    isaacbot_amr_battery_node = ComposableNode(
        package="isaacbot_amr_battery",
        plugin="isaacbot_amr_battery::IsaacbotBatteryNode",
        name="isaacbot_amr_battery_node",
        namespace=namespace,
        parameters=[isaacbot_amr_battery_config_path],
        remappings=[("/diagnostics", "diagnostics")],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    isaacbot_amr_battery_container = ComposableNodeContainer(
        package="rclcpp_components",
        executable="component_container",
        name="isaacbot_amr_battery_container",
        namespace=namespace,
        composable_node_descriptions=[isaacbot_amr_udp_battery_receiver_node, isaacbot_amr_battery_node],
        arguments=[
            "--ros-args",
            "--log-level",
            log_level,
            "--log-level",
            limit_log_level_to_info("rcl", log_level),
        ],
        emulate_tty=True,
    )

    actions = [
        declare_log_level_arg,
        declare_namespace_arg,
        declare_isaacbot_amr_battery_config_path_arg,
        isaacbot_amr_battery_container,
    ]

    return LaunchDescription(actions)
