#!/usr/bin/env python

# Copyright 1996-2023 Cyberbotics Ltd.
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

"""Launch Webots Mavic 2 Pro driver."""

import os
import pathlib
import launch
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.utils import controller_url_prefix


def get_ros2_nodes(*args):
    package_dir = get_package_share_directory('webots_ros2_conveyor')

    conveyors = ["conveyor_1", "conveyor_2", "conveyor_3"]
    cambots = ["cambot_1", "cambot_2", "cambot_3"]
    robots = conveyors + cambots

    robot_descriptions = [pathlib.Path(os.path.join(
        package_dir, 'resource', f"{x}.urdf")).read_text() for x in robots]

    robots_nodes = []
    for index, robot_description in enumerate(robot_descriptions):
        robots_nodes.append(
            Node(
                package='webots_ros2_driver',
                namespace=f"{robots[index]}",
                executable='driver',
                output='screen',
                additional_env={
                    'WEBOTS_CONTROLLER_URL': controller_url_prefix() + f"{robots[index]}"},
                parameters=[
                    {'robot_description': robot_description},
                ]
            ))

    return robots_nodes


def generate_launch_description():
    package_dir = get_package_share_directory('webots_ros2_conveyor')
    world = LaunchConfiguration('world')

    webots = WebotsLauncher(
        world=PathJoinSubstitution([package_dir, 'worlds', world]),
        ros2_supervisor=True
    )

    # The following line is important!
    # This event handler respawns the ROS 2 nodes on simulation reset (supervisor process ends).
    reset_handler = launch.actions.RegisterEventHandler(
        event_handler=launch.event_handlers.OnProcessExit(
            target_action=webots._supervisor,
            on_exit=get_ros2_nodes,
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='ure.wbt',
            description='Choose one of the world files from `/webots_ros2_conveyor/worlds` directory'
        ),
        webots,
        webots._supervisor,

        # This action will kill all nodes once the Webots simulation has exited
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[
                    launch.actions.UnregisterEventHandler(
                        event_handler=reset_handler.event_handler
                    ),
                    launch.actions.EmitEvent(event=launch.events.Shutdown())
                ],
            )
        ),

        # Add the reset event handler
        reset_handler
    ] + get_ros2_nodes())
