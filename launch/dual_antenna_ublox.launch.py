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
from ament_index_python.packages import get_package_share_directory
import launch
from launch_ros.actions import Node


def generate_launch_description():
    config_directory = os.path.join(
        get_package_share_directory('atb_ublox_gps'),
        'config')
    rover_params = os.path.join(config_directory, 'zed_f9p_rover.yaml')
    ublox_rover_node = Node(package='ublox_gps',
                                             name='rover',
                                             executable='ublox_gps_node',
                                             output='both',
                                             parameters=[rover_params])
    
    base_params = os.path.join(config_directory, 'zed_f9p_base.yaml')
    ublox_base_node = Node(package='ublox_gps',
                                            name='base',
                                             executable='ublox_gps_node',
                                             output='both',
                                             parameters=[base_params],
                                             remappings=[('/rtcm', '/ntrip_client/rtcm')]
                                             )
    ntrip_client_node = Node(package='ublox_gps',
                                        name='base',
                                            executable='ublox_gps_node',
                                            output='both',
                                            parameters=[base_params],
                                            remappings=[('/rtcm', '/ntrip_client/rtcm')]
                                            )


    return launch.LaunchDescription([ublox_rover_node,
                                     ublox_base_node,
                                     IncludeLaunchDescription(
                                        PythonLaunchDescriptionSource([
                                            FindPackageShare("ntrip_client"), '/launch', '/gazebo.launch.py'])
            )
                                     ])



# <launch>
#   <arg name="output"              default="screen" />
#   <arg name="respawn"             default="true" />
#   <arg name="respawn_delay"       default="30" />
#   <arg name="clear_params"        default="true" />

#   <node pkg="ublox_gps" type="ublox_gps" name="rover"
#         output="$(arg output)"
#         clear_params="$(arg clear_params)"
#         respawn="$(arg respawn)"
#         respawn_delay="$(arg respawn_delay)">
#     <rosparam command="load"
#               file="$(find ublox_gps)/config/zed_f9p_rover.yaml" />
#   </node>

#   <node pkg="ublox_gps" type="ublox_gps" name="base"
#         output="$(arg output)"
#         clear_params="$(arg clear_params)"
#         respawn="$(arg respawn)"
#         respawn_delay="$(arg respawn_delay)">
#     <rosparam command="load"
#               file="$(find ublox_gps)/config/zed_f9p_base.yaml" />
#     <remap from="/rtcm" to="/ntrip_client/rtcm" />
#   </node>
# </launch>
