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
from launch.actions import IncludeLaunchDescription
from launch.substitutions import EnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros.actions

from webots_ros2_core.utils import ControllerLauncher
from webots_ros2_core.webots_launcher import WebotsLauncher
from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import ReplaceString

def generate_launch_description():
    webots = WebotsLauncher(
        world=os.path.join(get_package_share_directory('thymio_example'), 'worlds',
                           'unicorn_pucks_enu.wbt')
    )

    stop_webots = launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )

    webots_nodes = [webots, stop_webots]

    thymio1 = make_thymio('thymio1')
    thymio2 = make_thymio('thymio2')
    thymio3 = make_thymio('thymio3')
    thymio4 = make_thymio('thymio4')

    nodes = webots_nodes + thymio1 + thymio2 + thymio3 + thymio4

    return launch.LaunchDescription(nodes)

def make_thymio(name):
    # Controller node
    synchronization = launch.substitutions.LaunchConfiguration('synchronization', default=False)
    controller = ControllerLauncher(
        package='thymio_controller',
        additional_env= {'WEBOTS_ROBOT_NAME': name},
        executable='thymio_controller',
        namespace=name,
        parameters=[{'synchronization': synchronization,
                     'use_joint_state_publisher': True}],
        output='screen',
        remappings=[("/tf", "tf")] # make sure tf gets namespaced
    )

    # nav2 stack
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_launch_dir = os.path.join(nav2_bringup_dir, 'launch')

    map_yaml = os.path.join(get_package_share_directory('thymio_example'), 'maps', 'ros_example.yaml')
    nav2_params = os.path.join(get_package_share_directory('thymio_example'), 'config', 'nav2_params.yaml')
    nav2_bt_xml = os.path.join(get_package_share_directory('nav2_bt_navigator'),
                          'behavior_trees', 'navigate_w_replanning_and_recovery.xml')

    rviz_config = os.path.join(get_package_share_directory('thymio_example'), 'config', 'thymio_view.rviz')

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_launch_dir, 'bringup_launch.py')),
        launch_arguments={'namespace': name,
                          'use_namespace': 'True',
                          'slam': 'False',
                          'map': map_yaml,
                          'use_sim_time': 'False',
                          'params_file': nav2_params,
                          'default_bt_xml_filename': nav2_bt_xml,
                          'autostart': 'True'}.items())

    rviz_config = ReplaceString(
        source_file=rviz_config,
        replacements={'<robot_namespace>': ('/' + name)})

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_launch_dir, 'rviz_launch.py')),
        launch_arguments={'namespace': name,
                          'use_namespace': 'True',
                          'rviz_config': rviz_config}.items())
    
    return [controller] #, nav2, rviz]
