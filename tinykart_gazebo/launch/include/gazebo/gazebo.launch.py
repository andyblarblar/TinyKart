import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import launch.substitutions
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():
    # ROS packages
    pkg_tinykart_description = get_package_share_directory(
        'tinykart_description')
    pkg_tinykart_gazebo = get_package_share_directory('tinykart_gazebo')
    pkg_ros_ign_gazebo = get_package_share_directory('ros_ign_gazebo')

    gazebo_world = LaunchConfiguration(
        'gazebo_world', default='tinykarts_world_shapes.sdf')

    # Nodes
    ign_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_ign_gazebo, 'launch',
                         'ign_gazebo.launch.py')),
        launch_arguments={
            'ign_args': launch.substitutions.PathJoinSubstitution([pkg_tinykart_gazebo + '/worlds/', gazebo_world])
        }.items()
    )

    ign_bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        arguments=[
            '/world/tinykarts_world/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/model/tinykart/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            '/model/tinykart/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            '/model/tinykart/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '/model/tinykart/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model',
            '/lidar@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/lidar/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
            '/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
            '/rgbd_camera/image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/rgbd_camera/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/rgbd_camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
            '/rgbd_camera/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
        ],
        output='screen',
        remappings=[
            ('/world/tinykarts_world/clock', '/clock'),
            ('/model/tinykart/tf', '/tf'),
            ('/model/tinykart/cmd_vel', '/robot/cmd_vel'),
            ('/model/tinykart/odometry', '/tinykart/odom'),
            ('/lidar', '/lidar/unfiltered_scan'),
            ('/imu', '/tinykart/imu'),
            ('/model/tinykart/joint_state', 'joint_states')
        ])

    #('/model/tinykart/joint_state', 'joint_states'),
    ign_spawn_robot = Node(package='ros_ign_gazebo',
                           executable='create',
                           arguments=[
                               '-name', 'tinykart', '-x', '0', '-z', '0', '-Y',
                               '0', '-topic', 'robot_description'
                           ],
                           output='screen')

    return LaunchDescription([
        # Nodes
        ign_gazebo,
        ign_bridge,
        ign_spawn_robot,

        DeclareLaunchArgument('gazebo_world',
                              default_value='tinykarts_word_shapes.sdf',
                              description='gazebo world to load'),
    ])
