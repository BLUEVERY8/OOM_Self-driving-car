#!/usr/bin/env python3
#
# Copyright 2023. Prof. Jong Min Lee @ Dong-eui University
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
#

import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import json
from launch.substitutions import LaunchConfiguration


print(os.path.realpath(__file__))

ld = LaunchDescription()
car = 'PR001'

def generate_launch_description():
    # configuration
    # 월드 파일 설정
    world = LaunchConfiguration('world')
    print('world =', world)
    world_file_name = 'test_track.world'
    world = os.path.join(get_package_share_directory('ros2_term_project'),
                         'worlds', world_file_name)
    print('world file name = %s' % world)

    # 가제보 내 시간 사용
    declare_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    # 가제보 실행
    gazebo_run = ExecuteProcess(
        cmd=['gazebo', '-s', 'libgazebo_ros_factory.so', world],
        output='screen')

    # 지정된 차량 정보를 전달하는 노드
    starter = Node(
        package='ros2_term_project',
        executable='starter',
        name='starter',
        arguments=[car],
        output='screen'
    )

    # 차량의 주행을 관리하는 노드
    controller = Node(
        package='ros2_term_project',
        executable='controller',
        name='controller',
        output='screen'
    )

    # 차선, 정지선, 종료선 등에 관한 정보를 처리하는 노드
    line_follower = Node(
        package='ros2_term_project',
        executable='line_follower',
        name='line_follower',
        arguments=[car],
        output='screen'
    )

    # 실행 목록에 추가
    ld.add_action(declare_argument)
    ld.add_action(gazebo_run)
    ld.add_action(controller)
    ld.add_action(line_follower)
    ld.add_action(starter)

    return ld
