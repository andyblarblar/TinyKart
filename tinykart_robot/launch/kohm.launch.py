# MIT License
#
# Copyright (c) 2021 Intelligent Systems Club
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition

#TODO this file wont launch, stuff like EKF isn't done
def generate_launch_description():
    # ROS packages
    pkg_tinykart_robot = get_package_share_directory('tinykart_robot')
    pkg_robot_state_controller = get_package_share_directory('robot_state_controller')
    pkg_teleop_twist_joy = get_package_share_directory('teleop_twist_joy')

    # Config
    joy_config = os.path.join(pkg_tinykart_robot, 'config/joystick',
                              'wii-wheel.config.yaml')

    # Launch arguments
    drive_mode_switch_button = LaunchConfiguration('drive_mode_switch_button', default='7')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    
    # Publishes joint and tf things
    state_publishers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_tinykart_robot, 'launch'),
            '/include/state_publishers/state_publishers.launch.py'
        ]),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )    
    
    # Provides cmd vel from joy inputs
    joy_with_teleop_twist = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_teleop_twist_joy, 'launch', 'teleop-launch.py')),
        launch_arguments={
            'joy_config': 'xbox',
            'joy_dev': '/dev/input/js0',
            'config_filepath': joy_config
        }.items(),
    )

    # Launches rviz
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_tinykart_robot, 'launch'),
            '/include/rviz/rviz.launch.py'
        ]),
        launch_arguments={
            'use_rviz': use_rviz,
            'use_sim_time': use_sim_time
        }.items(),
    )
    
    # The entire nav2 + slamtoolbox stack lol
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_tinykart_robot, 'launch'),
            '/include/navigation/navigation.launch.py'
        ]),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    # Followes waypoints set in a csv
    waypoint_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_tinykart_robot, 'launch'),
            '/include/waypoint_publisher/waypoint.launch.py'
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'follow_waypoints': True,
            'follow_gps' : False
        }.items(),
    )

    # Provides a state machine, and the node that provides the initalpose when a joy button is pressed.
    # This publish is what starts auton.
    robot_state_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_robot_state_controller, 'launch'),
            '/rsc_with_ipp.launch.py'
        ]),
        launch_arguments={
            'switch_button': drive_mode_switch_button,
            'use_sim_time': use_sim_time
        }.items(),
    )

    robot_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_tinykart_robot, 'launch/include/robot_localization'),
            '/localization.launch.py'
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items(),
    )

    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument(
            'drive_mode_switch_button',
            default_value='10',
            description='Which button is used on the joystick to switch drive mode. (In joy message)'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument('use_rviz',
                              default_value='true',
                              description='Open rviz if true'),

        # Nodes
        state_publishers,
        joy_with_teleop_twist,
        
        navigation,
        rviz,
        waypoint_publisher,
        robot_state_controller,
        robot_localization,
    ])
