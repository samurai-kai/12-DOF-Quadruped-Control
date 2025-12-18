from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_share = get_package_share_directory('rl12dof_urdf_description')

    config = PathJoinSubstitution([
        pkg_share,
        'config',
        'IK_cmds.yaml'
    ])

    return LaunchDescription([
        Node(
            package='rl12dof_urdf_description',
            executable='IKTestNode.py',
            name='IKTestNode',
            output='screen',
            parameters=[config]
        )
    ])
