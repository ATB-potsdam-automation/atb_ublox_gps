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


import os
import xacro
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from ament_index_python.packages import get_package_share_path

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

    ntrip_params = PathJoinSubstitution([
                FindPackageShare('atb_ublox_gps'),
                'config',
                'params.yaml'
                ])
    
    
    ntrip_client_node = Node(
                name='ntrip_client_node',
                namespace='ntrip_client',
                package='ntrip_client',
                executable='ntrip_ros',
                output= 'both',
                parameters=[ntrip_params],
                remappings=[
                 ("/ntrip_client/nmea", "/rover/nmea")
                ],
          )
    parameters_file_path = PathJoinSubstitution([
                FindPackageShare('atb_ublox_gps'),
        'config', 'xsens_mti_node.yaml'])
    xsens_mti_node = Node(
            package='bluespace_ai_xsens_mti_driver',
            executable='xsens_mti_node',
            name='xsens_mti_node',
            output='both',
            parameters=[parameters_file_path],
            arguments=[]
            )
    urdf_path = os.path.join(get_package_share_path('atb_ublox_gps'), 'config',
                             'urdf', 'sensor_traeger.urdf.xacro')
    robot_urdf = xacro.process_file(urdf_path)
    
    robot_state_publisher_node= Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
        'robot_description':  robot_urdf.toxml(), 
        'use_sim_time' : True 
    }])



    
    parameters_file_path =  PathJoinSubstitution([
                FindPackageShare('atb_ublox_gps'),
        'config', 'dual_ekf_navsat_example.yaml'])
    
    #ekf_odom_node = Node(
    #        package='robot_localization', 
    #        executable='ekf_node', 
    #        name='ekf_filter_node_odom',
	#        output='screen',
    #        parameters=[parameters_file_path],
    #        remappings=[('odometry/filtered', '/odometry/local')]           
    #       )
    ekf_map_node = Node(
            package='robot_localization', 
            executable='ekf_node', 
            name='ekf_filter_node_map',
	        output='screen',
            parameters=[parameters_file_path],
            remappings=[('odometry/filtered', '/odometry/global')]
           )          
    navsat_node = Node(
            package='robot_localization', 
            executable='navsat_transform_node', 
            name='navsat_transform',
	        output='screen',
            parameters=[parameters_file_path],
            remappings=[('/imu', '/imu/data'),
                        ('/gps/fix', '/rover/gps_node/fix'), 
                        ('odometry/gps', 'odometry/gps'),
                        ('/odometry/filtered', '/odometry/global')]           

           )  

    return LaunchDescription([ublox_rover_node,
                                     ublox_base_node,
                                     ntrip_client_node,
                                     xsens_mti_node,
                                     robot_state_publisher_node,
                                     ekf_map_node,
                                     navsat_node
                                     ])

