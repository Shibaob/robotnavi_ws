import os
import yaml

from ament_index_python.packages import get_package_share_directory, get_package_prefix

from launch import LaunchDescription, LaunchContext
from launch.actions import IncludeLaunchDescription, GroupAction, DeclareLaunchArgument, Shutdown, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, PushRosNamespace, RosTimer, SetUseSimTime

def robot_controller_actions(context : LaunchContext):

    num_robots = int(context.launch_configurations['num_robots'])
        
    yaml_path = os.path.join(get_package_share_directory('assessment'), 'config', 'initial_poses.yaml')

    with open(yaml_path, 'r') as f:
        configuration = yaml.safe_load(f)

    initial_poses = configuration[num_robots]

    actions = []

    for robot_number in range(1, num_robots + 1):

        robot_name = 'robot' + str(robot_number)

        group = GroupAction([
            PushRosNamespace("/" + robot_name),

            Node(
                package='sol',
                executable='target_finder',
                output='screen'),
            Node(
                package='sol',
                executable='target_decider',
                output='screen'),
            Node(
                package='sol',
                executable='proximity_detector',
                output='screen'),
            Node(
                package='sol',
                executable='navigation_manager',
                output='screen',
                parameters=[initial_poses[robot_name]])
        ])

        actions.append(group)

    return actions

def generate_launch_description():

    num_robots = LaunchConfiguration('num_robots')
    use_rviz = LaunchConfiguration('use_rviz')
    random_seed = LaunchConfiguration('random_seed')
    experiment_duration = LaunchConfiguration('experiment_duration')

    declare_num_robots_cmd = DeclareLaunchArgument(
        'num_robots',
        default_value='1',
        description='Number of robots to spawn')
    
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='If rViz is launched with the solution')
    
    declare_random_seed_cmd = DeclareLaunchArgument(
        'random_seed',
        default_value='0',
        description='Random number seed for item manager')
    
    declare_experiment_duration_cmd = DeclareLaunchArgument(
        'experiment_duration',
        default_value='300.0',
        description='Experiment duration in seconds')
    
    rviz_config = PathJoinSubstitution([FindPackageShare('sol'), 'rviz', 'config.rviz'])
    rviz_windows = PathJoinSubstitution([FindPackageShare('assessment'), 'config', 'rviz_windows.yaml'])

    map = PathJoinSubstitution([FindPackageShare('assessment'), 'maps', 'assessment_world.yaml'])
    params = PathJoinSubstitution([FindPackageShare('sol'), 'params', 'custom_parameters.yaml'])

    assessment_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('assessment'),
                'launch',
                'assessment_launch.py'
                ])
        ),
        launch_arguments={'num_robots': num_robots,
                          'visualise_sensors': 'false',
                          'odometry_source': 'ENCODER',
                          'sensor_noise': 'false',
                          'use_rviz': use_rviz,
                          'rviz_config': rviz_config,
                          'rviz_windows': rviz_windows,
                          'obstacles': 'true',
                          'item_manager': 'true',
                          'random_seed': random_seed,
                          'use_nav2': 'True',
                          'map': map,
                          'params_file': params,
                          'headless': 'false',
                          'limit_real_time_factor': 'true',
                          'wait_for_items': 'false',
                          }.items()
    )

    robot_controller_cmd = OpaqueFunction(function=robot_controller_actions)

    timeout_cmd = RosTimer(                                         
        period = experiment_duration,
        actions = [                                                       
            Shutdown(reason="Experiment timeout reached")     
        ],
    )

    multi_node_cmd = Node(
        package='sol',
        executable='multi_manager',
        output='screen',
        parameters=[{"num_robots":num_robots}]
    )

    ld = LaunchDescription()

    ld.add_action(SetUseSimTime(True))

    ld.add_action(declare_num_robots_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_random_seed_cmd)
    ld.add_action(declare_experiment_duration_cmd)

    ld.add_action(assessment_cmd)
    ld.add_action(robot_controller_cmd)
    ld.add_action(multi_node_cmd)
    ld.add_action(timeout_cmd)

    return ld
