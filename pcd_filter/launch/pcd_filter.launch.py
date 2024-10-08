# Copyright 2024 CLOBOT Co., Ltd
# 
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
# 
#     https://www.apache.org/licenses/LICENSE-2.0
# 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

import ament_index_python.packages
import launch
import launch_ros.actions
import subprocess


def generate_launch_description():
  return launch.LaunchDescription([
    Node(
      package='pcd_filter',
      executable='pointcloud_filter_node',
      name='pcd_filter_node',
      output='screen'
    )
  ])



