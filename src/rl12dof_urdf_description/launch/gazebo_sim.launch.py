from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterValue
import os
import xacro
import datetime
import shutil  # Add this import
import uuid  # Add this import
from launch.substitutions import Command
from launch.actions import TimerAction

def generate_launch_description():
    # Package and path definitions
    pkg_ros_gz_sim = FindPackageShare('ros_gz_sim').find('ros_gz_sim')
    pkg_ros_gz_bridge = FindPackageShare('ros_gz_bridge').find('ros_gz_bridge')

    # Clock bridge node
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'qos_overrides./clock.publisher.durability': 'transient_local',
            'qos_overrides./clock.publisher.reliability': 'reliable',
        }],
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'] 
    )

    # Odometry bridge node
    state_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='state_bridge',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[
            # Bridge Gazebo odometry to ROS topic
            '/model/quadruped/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',  
        ],
        remappings=[
            ('/model/quadruped/odometry' , '/quadruped/state/ground_truth/odometry')  # remap Gazebo publication
        ]
    )

    # Robot description publisher
    urdf_file = os.path.join(
        FindPackageShare('rl12dof_urdf_description').find('rl12dof_urdf_description'),
        'urdf', 'rl12dof_urdf.xacro'
    )
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
    )
    
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            os.path.join(
                FindPackageShare('rl12dof_urdf_description').find('rl12dof_urdf_description'),
                'config', 'controllers.yaml'
            ),
            {'use_sim_time': True},
            # {'robot_description': ParameterValue(Command(['xacro ', urdf_file]), value_type=str)},
        ],
        output="screen",
    )
    
    # Spawn robot node
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_robot_urdf',
        output='screen',
        arguments=['-topic', 'robot_description',
                   '-x', '0.0', '-y', '0.0', '-z', '.3',
                   '-R', '0.0', '-P', '0.0', '-Y', '0.0',
                   '-entity', 'Edward'],
        parameters=[{'use_sim_time': True, 'update_rate': 1000}]
    )

    # PlotJuggler node with only auto-subscribe
    plotjuggler = Node(
        package='plotjuggler',
        executable='plotjuggler',
        name='plotjuggler',
        parameters=[{'use_sim_time': True}]
    )

    # Configure storage location for ROS2 bags with unique timestamp
    base_path = os.path.join(os.getcwd(), 'src', 'quadruped_sim', 'logs')
    if os.path.exists(base_path):
        shutil.rmtree(base_path)
    timestamp = datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
    unique_id = str(uuid.uuid4())[:8]
    storage_path = os.path.join(base_path, f"{timestamp}-{unique_id}")
    os.makedirs(storage_path)

    # ROS2 bag recording node with only pose data
    bag_recorder = ExecuteProcess(
        cmd=['ros2', 'bag', 'record',
             '--output', storage_path,
             '/world/empty/pose/info'],
        output='screen'
    )

    return LaunchDescription([
        # Start Gazebo FIRST
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={
                'paused': 'true',
                'gui': 'true',
                'recording': 'false',
                'debug': 'false',
                'verbose': 'true',
                'gz_args': '-r empty.sdf',
            }.items()
        ),

        clock_bridge,

        # Publish robot_description
        robot_state_publisher,

        # Start ros2_control_node AFTER robot_description exists
        TimerAction(
            period=1.0,
            actions=[ros2_control_node],
        ),

        # Spawn controllers (must run AFTER ros2_control_node)
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=["joint_state_broadcaster"],
                    output="screen"
                ),
            ],
        ),

        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=["joint_group_position_controller"],
                    output="screen"
                ),
            ],
        ),

        # NOW spawn robot in Gazebo
        TimerAction(
            period=4.0,
            actions=[spawn_robot],
        ),

        # Optional bridge + bag
        state_bridge,
        bag_recorder,
    ])
