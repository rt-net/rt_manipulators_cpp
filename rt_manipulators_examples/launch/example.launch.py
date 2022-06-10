# Copyright 2022 RT Corporation
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

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    declare_example_name = DeclareLaunchArgument(
        'example', default_value='x7_forward_kinematics',
        description=('Set an example executable name: '
                     '[x7_forward_kinematics]')
    )

    config_file_path = os.path.join(
        get_package_share_directory('rt_manipulators_examples'),
        'config',
        'crane-x7.yaml'
    )

    link_file_path = os.path.join(
        get_package_share_directory('rt_manipulators_examples'),
        'config',
        'crane-x7_links.csv'
    )

    example_node = Node(name=[LaunchConfiguration('example')],
                        package='rt_manipulators_examples',
                        executable=LaunchConfiguration('example'),
                        output='screen',
                        parameters=[{
                            'port_name': '/dev/ttyUSB0',
                            'baudrate': 3000000,
                            'config_file_path': config_file_path,
                            'link_file_path': link_file_path,
                        }])

    ld = LaunchDescription()
    ld.add_action(declare_example_name)
    ld.add_action(example_node)

    return ld
