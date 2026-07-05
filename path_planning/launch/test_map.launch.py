import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG_SHARE = get_package_share_directory('path_planning')
CONFIG    = os.path.join(PKG_SHARE, 'config', 'caltech_scan.yaml')
# CONFIG    = os.path.join(PKG_SHARE, 'config', 'mapping.yaml')
RVIZ_CFG  = os.path.join(PKG_SHARE, 'rviz',   'rviz.rviz')


def generate_launch_description():
    pcd_arg = DeclareLaunchArgument(
        'pcd', default_value='', description='Path to .pcd file')

    test_node = Node(
        package='path_planning',
        executable='test_map',
        name='test_map',
        output='screen',
        arguments=[LaunchConfiguration('pcd')],
        parameters=[CONFIG],
    )

    rviz_args = ['-d', RVIZ_CFG] if os.path.isfile(RVIZ_CFG) else []
    rviz_node = ExecuteProcess(
        cmd=['rviz2'] + rviz_args,
        output='screen',
    )

    return LaunchDescription([pcd_arg, test_node, rviz_node])
