from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    config = PathJoinSubstitution([
        FindPackageShare('rl12dof_urdf_description'),
        'config',
        'FK_cmds.yaml'
    ])

    return LaunchDescription([
        Node(
            package='rl12dof_urdf_description',
            executable='FKTestNode.py',
            name='FKTestNode',
            parameters=[config],
            output='screen'
        )
    ])
