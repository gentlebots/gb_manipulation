# Copyright 2019 Intelligent Robotics Lab
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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    pkg_dir = get_package_share_directory('gb_manipulation')
    namespace = LaunchConfiguration('namespace')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace')

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1')

    # Specify the actions
    pick_1_cmd = Node(
        package='plansys2_bt_actions',
        executable='bt_action_node',
        name='pick_1',
        namespace=namespace,
        output='screen',
        parameters=[
          pkg_dir + '/config/params.yaml',
          {
            'action_name': 'pick',
            'bt_xml_file': pkg_dir + '/behavior_trees_xml/pick.xml'
          }
        ])
    place_1_cmd = Node(
        package='plansys2_bt_actions',
        executable='bt_action_node',
        name='place_1',
        namespace=namespace,
        output='screen',
        parameters=[
          pkg_dir + '/config/params.yaml',
          {
            'action_name': 'place',
            'bt_xml_file': pkg_dir + '/behavior_trees_xml/place.xml'
          }
        ])

    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(declare_namespace_cmd)
    
    # Declare the launch options
    ld.add_action(pick_1_cmd)
    ld.add_action(place_1_cmd)
  
    return ld
