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


def generate_launch_description():
    # Webots
    webots = WebotsLauncher(
        world=os.path.join(get_package_share_directory('thymio_example'), 'worlds',
                           'ros_example.wbt')
    )

    # Controller node
    synchronization = launch.substitutions.LaunchConfiguration('synchronization', default=False)
    controller_1 = ControllerLauncher(
        package='thymio_controller',
        additional_env= {'WEBOTS_ROBOT_NAME': 'Thymio II'},
        executable='thymio_controller',
        namespace='/thymio1',
        parameters=[{'synchronization': synchronization,
                     'use_joint_state_publisher': True}],
        output='screen',
        remappings=[("/tf", "/thymio1/tf")]
    )

    controller_2 = ControllerLauncher(
        package='thymio_controller',
        additional_env= {'WEBOTS_ROBOT_NAME': 'Thymio II_2'},
        executable='thymio_controller',
        namespace='/thymio2',
        parameters=[{'synchronization': synchronization,
                     'use_joint_state_publisher': True}],
        output='screen',
        remappings=[("/tf", "/thymio2/tf")]        
    )

    # nav2 stack
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_launch_dir = os.path.join(nav2_bringup_dir, 'launch')

    map_yaml = os.path.join(get_package_share_directory('thymio_example'), 'maps', 'ros_example.yaml')
    # nav2_params = os.path.join(nav2_bringup_dir, 'params', 'nav2_params.yaml')
    nav2_params = os.path.join(get_package_share_directory('thymio_example'), 'config', 'nav2_params.yaml')
    nav2_bt_xml = os.path.join(get_package_share_directory('nav2_bt_navigator'),
                          'behavior_trees', 'navigate_w_replanning_and_recovery.xml')
    nav2_1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_launch_dir, 'bringup_launch.py')),
        launch_arguments={'namespace': '/thymio1',
                          'use_namespace': 'True',
                          'slam': 'False',
                          'map': map_yaml,
                          'use_sim_time': 'False',
                          'params_file': nav2_params,
                          'default_bt_xml_filename': nav2_bt_xml,
                          'autostart': 'True'}.items())

    nav2_2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_launch_dir, 'bringup_launch.py')),
        launch_arguments={'namespace': '/thymio2',
                          'use_namespace': 'True',
                          'slam': 'False',
                          'map': map_yaml,
                          'use_sim_time': 'False',
                          'params_file': nav2_params,
                          'default_bt_xml_filename': nav2_bt_xml,
                          'autostart': 'True'}.items())
    

    
    # rviz 
    rviz_config = os.path.join(get_package_share_directory('thymio_example'), 'config', 'ros_example2.rviz')
    rviz_1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_launch_dir, 'rviz_launch.py')),
        launch_arguments={'namespace': '/thymio1',
                          'use_namespace': 'True',
                          'rviz_config': rviz_config}.items())

    rviz_2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_launch_dir, 'rviz_launch.py')),
        launch_arguments={'namespace': '/thymio2',
                          'use_namespace': 'True',
                          'rviz_config': rviz_config}.items())
    

    return launch.LaunchDescription([
        webots,
        controller_1,
        controller_2,
        nav2_1,
        nav2_2,
        rviz_1,
        rviz_2,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        ),
    ])
