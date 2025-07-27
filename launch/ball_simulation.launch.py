#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # パッケージのディレクトリを取得
    pkg_ball_gazebo_sim = get_package_share_directory('ball_gazebo_sim')
    
    # 世界ファイルのパス
    world_file_path = os.path.join(pkg_ball_gazebo_sim, 'worlds', 'ball.sdf')
    
    # Gazebo起動の設定
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={
            'gz_args': ['-r -v 4 ', world_file_path]
        }.items()
    )

    # ROS-Gazeboブリッジノード（X軸ジョイント制御）
    bridge_x_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_x',
        arguments=[
            '/joint_commands@std_msgs/msg/Float64@gz.msgs.Double'
        ],
        output='screen'
    )

    # ROS-Gazeboブリッジノード（Y軸ジョイント制御）
    bridge_y_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_y',
        arguments=[
            '/joint_commands_y@std_msgs/msg/Float64@gz.msgs.Double'
        ],
        output='screen'
    )

    # ROS-Gazeboブリッジノード（ボールポーズ情報 - 名前付きで取得）
    bridge_pose_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_pose',
        arguments=[
            '/world/ball_world/pose/info@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V'
        ],
        output='screen'
    )

    # ボール専用ブリッジノード
    bridge_ball_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_ball',
        arguments=[
            '/model/ball/pose@geometry_msgs/msg/Pose@gz.msgs.Pose'
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo_launch,
        bridge_x_node,
        bridge_y_node,
        bridge_pose_node,
        bridge_ball_node
    ]) 