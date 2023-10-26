# Copyright (c) 2018 Intel Corporation
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
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import PushRosNamespace
from launch_ros.actions import Node

def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('ocs2_cartpole_ros')
    launch_dir = os.path.join(bringup_dir, 'launch')

    # Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the navigation stack')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'params', 'controller_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    rviz_config = os.path.join(bringup_dir, 'rviz', 'cartpole.rviz')

    # Launch rviz
    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        output='screen')

    # Launch cartpole urdf
    cartpole_urdf_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'visualize.launch.py')),
        launch_arguments={'namespace': namespace,
                          'use_namespace': use_namespace}.items())

    multiplot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'multiplot.launch.py')),
        launch_arguments={'namespace': namespace,
                          'use_namespace': use_namespace}.items())

    # Launch cartpole_mpc
    start_cartpole_mpc_cmd = Node(
        package='ocs2_cartpole_ros',
        executable='cartpole_mpc',
        name='cartpole_mpc',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}])

    # Launch cartpole_mpc
    start_cartpole_dummy_test_cmd = Node(
        package='ocs2_cartpole_ros',
        executable='cartpole_dummy_test',
        name='cartpole_dummy_test',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}])

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(start_rviz_cmd)
    ld.add_action(cartpole_urdf_cmd)
    # ld.add_action(start_cartpole_mpc_cmd)
    # ld.add_action(start_cartpole_dummy_test_cmd)

    return ld
