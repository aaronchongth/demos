# Copyright 2021 Open Source Robotics Foundation, Inc.
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

from ament_index_python.packages import get_package_share_directory

import os
import sys
import launch
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    """Generate launch description with multiple components."""
    log_level = 'info'

    config_file = os.path.join(
        get_package_share_directory("composition"),
        "config",
        "test.yaml",
    )

    container = ComposableNodeContainer(
        name='ComponentManager',
        package='rclcpp_components',
        executable='component_container_mt',
        namespace='dean',
        composable_node_descriptions=[
            ComposableNode(
                package='composition',
                plugin='composition::Listener',
                namespace='',
                name='listener',
                parameters = [
                    config_file,
                    # {"topic_name": "chatter2"}
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        emulate_tty=True,
    )
    nodes = [
        container,
    ]

    return launch.LaunchDescription(
        nodes
    )
