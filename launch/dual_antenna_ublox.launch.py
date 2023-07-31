# Copyright 2020 Open Source Robotics Foundation, Inc.
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# * Redistributions of source code must retain the above copyright
#   notice, this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above
#   copyright notice, this list of conditions and the following
#   disclaimer in the documentation and/or other materials provided
#   with the distribution.
# * Neither the name of {copyright_holder} nor the names of its
#   contributors may be used to endorse or promote products derived
#   from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Launch two ublox gps nodes with zed_f9p_rover and zed_f9p_base configuration."""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():   
    
    rover_params = PathJoinSubstitution([
                    FindPackageShare('atb_ublox_gps'),
                    'config',
                    'zed_f9p_rover.yaml'
                ])
    ublox_rover_node = Node(package='ublox_gps',
                                             name='gps_node',
                                             namespace='rover',
                                             executable='ublox_gps_node',
                                             output='both',
                                             parameters=[rover_params])
    
    base_params = PathJoinSubstitution([
                    FindPackageShare('atb_ublox_gps'),
                    'config',
                    'zed_f9p_base.yaml'
                ])
    
    ublox_base_node = Node(package='ublox_gps',
                                            name='gps_node',
                                            namespace='base',
                                            executable='ublox_gps_node',
                                            output='both',
                                            parameters=[base_params],
                                            remappings=[('/rtcm', '/ntrip_client/rtcm')]
                                            )


    return LaunchDescription([ublox_rover_node,
                                     ublox_base_node,
                                     IncludeLaunchDescription(
                                        PythonLaunchDescriptionSource([
                                            FindPackageShare("ntrip_client"), '/launch', '/ntrip_client.launch.py']))
                                     ])

