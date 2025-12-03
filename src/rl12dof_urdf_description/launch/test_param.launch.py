from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    config = PathJoinSubstitution([
        FindPackageShare('rl12dof_urdf_description'),
        'config',
        'joint_tests.yaml'
    ])

    return LaunchDescription([
        Node(
            package='rl12dof_urdf_description',
            executable='joint_test_param_node.py',
            name='joint_test_param_node',
            parameters=[config],
            output='screen'
        )
    ])
