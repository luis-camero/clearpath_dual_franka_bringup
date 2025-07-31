# Software License Agreement (BSD)
#
# @author    Luis Camero <lcamero@clearpathrobotics.com>
# @copyright (c) 2024, Clearpath Robotics, Inc., All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# * Redistributions of source code must retain the above copyright notice,
#   this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# * Neither the name of Clearpath Robotics nor the names of its contributors
#   may be used to endorse or promote products derived from this software
#   without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# Redistribution and use in source and binary forms, with or without
# modification, is not permitted without the express permission
# of Clearpath Robotics.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (Command, FindExecutable,
                                  PathJoinSubstitution, LaunchConfiguration)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Launch Configurations
    robot_urdf = LaunchConfiguration('robot_urdf')
    robot_description_command = LaunchConfiguration('robot_description_command')
    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace = LaunchConfiguration('namespace')
    franka_0_control = LaunchConfiguration('franka_0_control')
    franka_1_control = LaunchConfiguration('franka_1_control')

    # Launch Arguments
    arg_robot_urdf = DeclareLaunchArgument(
        'robot_urdf',
        default_value='/etc/clearpath/robot.urdf.xacro'
    )

    arg_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Robot namespace'
    )

    # Get URDF via xacro
    arg_robot_description_command = DeclareLaunchArgument(
        'robot_description_command',
        default_value=[
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            robot_urdf,
            ' ',
            'is_sim:=',
            use_sim_time,
            ' ',
            'namespace:=',
            namespace,
            ' ',
            'use_fake_hardware:=',
            'false',
            ' ',
            'use_manipulation_controllers:=',
            'false',
            ' ',
            'use_platform_controllers:=',
            'false',
            ' ',
            'arm_0_control:=',
            franka_0_control,
            ' ',
            'arm_1_control:=',
            franka_1_control
        ]
    )

    arg_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        choices=['true', 'false'],
        default_value='false',
        description='Use simulation time'
    )

    arg_franka_0_control = DeclareLaunchArgument(
        'franka_0_control',
        choices=['true', 'false'],
        default_value='true',
        description='Enable hardware interface for Franka arm_0'
    )

    arg_franka_1_control = DeclareLaunchArgument(
        'franka_1_control',
        choices=['true', 'false'],
        default_value='false',
        description='Enable hardware interface for Franka arm_1'
    )

    robot_description_content = ParameterValue(
        Command(robot_description_command),
        value_type=str
    )

    # Manipulator State Publisher
    manipulator_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': use_sim_time,
        }],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
            ('dynamic_joint_states',
                PathJoinSubstitution([
                    '/', namespace, 'platform', 'dynamic_joint_states'
                ])),
            ('joint_states',
                PathJoinSubstitution([
                    '/', namespace, 'platform', 'joint_states'
                ])),
        ]
    )

    ld = LaunchDescription()
    # Args
    ld.add_action(arg_use_sim_time)
    ld.add_action(arg_robot_urdf)
    ld.add_action(arg_namespace)
    ld.add_action(arg_franka_1_control)
    ld.add_action(arg_franka_0_control)
    ld.add_action(arg_robot_description_command)
    # Nodes
    ld.add_action(manipulator_state_publisher)
    return ld
