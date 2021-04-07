#!/usr/bin/env python

# Copyright 1996-2021 Cyberbotics Ltd.
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

"""Launch Webots and the controller."""

import os
import launch
from launch.actions import SetEnvironmentVariable
from launch.actions import UnsetEnvironmentVariable
from launch.substitutions import EnvironmentVariable

from webots_ros2_core.utils import ControllerLauncher
from webots_ros2_core.webots_launcher import WebotsLauncher
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Webots
    webots = WebotsLauncher(
        world=os.path.join(get_package_share_directory('thymio_example'), 'worlds', 'ros_example.wbt')
    )

    # Controller node
    synchronization = launch.substitutions.LaunchConfiguration('synchronization', default=False)
    controller1 = ControllerLauncher(
        package='thymio_controller',
        additional_env= {'WEBOTS_ROBOT_NAME': 'Thymio II'},
        executable='thymio_controller',
        namespace='/thymio1',
        parameters=[{'synchronization': synchronization}],
        output='screen'
    )

    controller2 = ControllerLauncher(
        package='thymio_controller',
        additional_env= {'WEBOTS_ROBOT_NAME': 'Thymio II_2'},
        executable='thymio_controller',
        namespace='/thymio2',
        parameters=[{'synchronization': synchronization}],
        output='screen'
    )

    return launch.LaunchDescription([
        webots,
        controller1,
        controller2,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        ),
    ])
