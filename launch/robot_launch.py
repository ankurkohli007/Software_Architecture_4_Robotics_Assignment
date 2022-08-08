#!/usr/bin/env python



"""Launch Webots and the controller."""
#from robot_launch in tiago webots:
import os
import pathlib
import launch
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, get_packages_with_prefixes
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from webots_ros2_driver.webots_launcher import WebotsLauncher

#from costmap filter:
from nav2_common.launch import RewrittenYaml

#from nav2_bringup:
from launch_ros.actions import PushRosNamespace
from launch.actions import( GroupAction,
                            IncludeLaunchDescription,
                            SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    optional_nodes = []
    #-----------------------
    package_dir = get_package_share_directory('sofar_assignment')
    launch_dir = os.path.join(package_dir, 'launch')
    costmap_filters_dir = package_dir
    #-----------------------

    # Used later for the costmap filter:
    lifecycle_nodes = ['filter_mask_server', 'costmap_filter_info_server']

    # Create the launch configuration variables for webots and tiago
   
    mode = LaunchConfiguration('mode')
    use_rviz = LaunchConfiguration('rviz', default=True)
    use_nav = LaunchConfiguration('nav', default=True)
    use_slam = LaunchConfiguration('slam', default=False)
    robot_description = pathlib.Path(os.path.join(package_dir, 'URDF', 'tiago_webots.urdf')).read_text()
    ros2_control_params = os.path.join(package_dir, 'resource', 'ros2_control.yml')
    
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    # Create the launch configuration variables for nav2
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    slam = LaunchConfiguration('slam')
    autostart = LaunchConfiguration('autostart')

   
    params_file =  LaunchConfiguration(
        'params_file',
        default=os.path.join(package_dir, 'params', 'nav2_params.yaml')
        )

    keepout_params_file = LaunchConfiguration(
        'keepout_params_file',
        default=os.path.join(package_dir, 'params', 'keepout_params.yaml')
        )
   
    world = LaunchConfiguration(
        'world',
        default=os.path.join( package_dir, 'worlds', 'default.wbt')
        )
    
    nav2_map = LaunchConfiguration(
        'map',
        default=os.path.join(package_dir, 'maps', 'map.yaml')
        )
    map_yaml_file = nav2_map
    
    mask_yaml_file = LaunchConfiguration(
       'mask',
        default=os.path.join(package_dir, 'maps', 'keepout_mask.yaml')
    )

    webots = WebotsLauncher(
        world=world,
        mode=mode
    )

    controller_manager_timeout = ['--controller-manager-timeout', '50']
    controller_manager_prefix = 'python.exe' if os.name == 'nt' else ''

    use_deprecated_spawner_py = 'ROS_DISTRO' in os.environ and os.environ['ROS_DISTRO'] == 'foxy'

    diffdrive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner' if not use_deprecated_spawner_py else 'spawner.py',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['diffdrive_controller'] + controller_manager_timeout,
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner' if not use_deprecated_spawner_py else 'spawner.py',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['joint_state_broadcaster'] + controller_manager_timeout,
    )

    mappings = [('/diffdrive_controller/cmd_vel_unstamped', '/cmd_vel')]
    if ('ROS_DISTRO' in os.environ and os.environ['ROS_DISTRO'] == 'rolling') or \
       ('ROS_REPO' in os.environ and os.environ['ROS_REPO'] == 'testing'):
        mappings.append(('/diffdrive_controller/odom', '/odom'))

    tiago_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        parameters=[
            {'robot_description': robot_description,
             'use_sim_time': use_sim_time,
             'set_robot_state_publisher': True},
            ros2_control_params
        ],
        remappings=mappings
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': '<robot name=""><link name=""/></robot>'
        }],
    )

    footprint_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
    )

    rviz_config = os.path.join(get_package_share_directory('sofar_assignment'), 'resource', 'default.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['--display-config=' + rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=launch.conditions.IfCondition(use_rviz)
    )

    slam_toolbox = Node(
        parameters=[{'use_sim_time': use_sim_time}],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        condition=launch.conditions.IfCondition(use_slam)
    )
    
    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the navigation stack')

    declare_slam_cmd = DeclareLaunchArgument(
        'slam',
        default_value='False',
        description='Whether run a SLAM')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=nav2_map,
        description='Full path to map yaml file to load')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(package_dir, 'params', 'nav2_params.yaml'),
        #default_value=os.path.join( package_dir, 'params', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')
    
    declare_keepout_params_file_cmd = DeclareLaunchArgument(
            'keepout_params_file',
            default_value=os.path.join(costmap_filters_dir,'params','keepout_params.yaml'),
            #default_value=os.path.join(costmap_filters_dir,'params', 'keepout_params.yaml'),
            description='Full path to the ROS2 parameters file to use')
    
    declare_mask_yaml_file_cmd = DeclareLaunchArgument(
            'mask',
            default_value=os.path.join(costmap_filters_dir,'maps','keepout_mask.yaml'),
            #default_value=os.path.join(costmap_filters_dir,'maps','keepout_mask.yaml'),
            description='Full path to filter mask yaml file to load')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')
    
    # Make re-written yaml
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': mask_yaml_file}

    configured_params = RewrittenYaml(
        source_file=keepout_params_file,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True)
    # Nodes launching commands

    start_lifecycle_manager_cmd = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_costmap_filters',
            namespace=namespace,
            output='screen',
            emulate_tty=True,  # https://github.com/ros2/launch/issues/188
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': lifecycle_nodes}])

    start_map_server_cmd = Node(
            package='nav2_map_server',
            executable='map_server',
            name='filter_mask_server',
            namespace=namespace,
            output='screen',
            emulate_tty=True,  # https://github.com/ros2/launch/issues/188
            parameters=[configured_params])

    start_costmap_filter_info_server_cmd = Node(
            package='nav2_map_server',
            executable='costmap_filter_info_server',
            name='costmap_filter_info_server',
            namespace=namespace,
            output='screen',
            emulate_tty=True,  # https://github.com/ros2/launch/issues/188
            parameters=[configured_params])

    # Specify the actions
    bringup_cmd_group = GroupAction([
        PushRosNamespace(
            condition=IfCondition(use_namespace),
            namespace=namespace),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'slam_launch.py')),
            condition=IfCondition(slam),
            launch_arguments={'namespace': namespace,
                              'use_sim_time': use_sim_time,
                              'autostart': autostart,
                              'params_file': params_file}.items()),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'localization_launch.py')),
            condition=IfCondition(PythonExpression(['not ', slam])),
            launch_arguments={'namespace': namespace,
                              'map': map_yaml_file,
                              'use_sim_time': use_sim_time,
                              'autostart': autostart,
                              'params_file': params_file,
                              'use_lifecycle_mgr': 'false'}.items()),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'navigation_launch.py')),
            launch_arguments={'namespace': namespace,
                              'use_sim_time': use_sim_time,
                              'autostart': autostart,
                              'params_file': params_file,
                              'use_lifecycle_mgr': 'false',
                              'map_subscribe_transient_local': 'true'}.items()),
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            #default_value='default.wbt',
            default_value=os.path.join( package_dir, 'worlds', 'default.wbt'),

            description='Choose one of the world files from `/webots_ros2_tiago/world` directory'
        ),
        DeclareLaunchArgument(
            'mode',
            default_value='realtime',
            description='Webots startup mode'
        ),

        
        declare_namespace_cmd,
        declare_use_namespace_cmd,
        stdout_linebuf_envvar,

        #costmap filter:
        declare_use_sim_time_cmd,
        declare_autostart_cmd,
        declare_keepout_params_file_cmd,
        declare_mask_yaml_file_cmd,
        start_lifecycle_manager_cmd,
        start_map_server_cmd,
        start_costmap_filter_info_server_cmd,

        #nav2_bringup:        
        declare_slam_cmd,
        declare_map_yaml_cmd,
        joint_state_broadcaster_spawner,
        diffdrive_controller_spawner,
        webots,
        rviz,
        robot_state_publisher,
        tiago_driver,
        footprint_publisher,
        slam_toolbox,  
        declare_params_file_cmd,
        bringup_cmd_group,


        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ] + optional_nodes)
