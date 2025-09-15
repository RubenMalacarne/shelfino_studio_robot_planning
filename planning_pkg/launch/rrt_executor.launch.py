#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, TimerAction
from launch.substitutions import LaunchConfiguration, TextSubstitution
import launch_ros
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the launch directory
    pkg_planning_pkg = get_package_share_directory('planning_pkg')
    
    # Declare launch arguments
    use_multiagent_arg = DeclareLaunchArgument(
        'use_multiagent',
        default_value='false',
        description='Enable multi-agent A*CA* planning instead of individual RRT'
    )
    
    rrt_step_size_arg = DeclareLaunchArgument(
        'rrt_step_size',
        default_value='0.5',
        description='RRT step size for tree expansion'
    )
    
    rrt_max_iterations_arg = DeclareLaunchArgument(
        'rrt_max_iterations',
        default_value='5000',
        description='Maximum number of RRT iterations'
    )
    
    rrt_goal_bias_arg = DeclareLaunchArgument(
        'rrt_goal_bias',
        default_value='0.1',
        description='RRT goal biasing probability'
    )
    
    astar_time_step_arg = DeclareLaunchArgument(
        'astar_time_step',
        default_value='0.1',
        description='Multi-agent A* time discretization step'
    )
    
    astar_collision_radius_arg = DeclareLaunchArgument(
        'astar_collision_radius',
        default_value='0.2',
        description='Agent collision radius for multi-agent planning'
    )

    # Re-mapping Node (provides /service_trigger_inflated service)
    re_mapping_node = launch_ros.actions.Node(
        package='planning_pkg',
        executable='re_mapping',
        name='re_mapping',
        output='screen'
    )

    # Nav2 Path Client Node (executes paths to move robots)
    nav2_pathclient_node = launch_ros.actions.Node(
        package='planning_pkg',
        executable='nav2_pathclient',
        name='nav2_pathclient',
        output='screen'
    )

    # Dubins Curve Converter Node (converts RRT paths to Dubins paths)
    dubins_converter_node = TimerAction(
        period=1.0,
        actions=[
            launch_ros.actions.Node(
                package='planning_pkg',
                executable='dubins_curve_converter',
                name='dubins_converter',
                output='screen'
            )
        ]
    )

    # RRT Planning Orchestrator Node
    rrt_orchestrator_node = launch_ros.actions.Node(
        package='planning_pkg',
        executable='path_orchestrator_rrt',
        name='rrt_planning_orchestrator',
        output='screen',
        parameters=[{
            'use_multiagent_planning': LaunchConfiguration('use_multiagent'),
            'rrt_step_size': LaunchConfiguration('rrt_step_size'),
            'rrt_max_iterations': LaunchConfiguration('rrt_max_iterations'),
            'rrt_goal_bias': LaunchConfiguration('rrt_goal_bias'),
            'astar_time_step': LaunchConfiguration('astar_time_step'),
            'astar_collision_radius': LaunchConfiguration('astar_collision_radius'),
        }],
        remappings=[
            ('/inflated_obstacles', '/inflated_obstacles'),
            ('/inflated_arena', '/inflated_arena'),
            ('/published_gates', '/published_gates'),
            ('/published_pos1', '/published_pos1'),
            ('/published_pos2', '/published_pos2'),
            ('/path_pos1_to_gates', '/path_pos1_to_gates'),
            ('/path_pos2_to_gates', '/path_pos2_to_gates'),
        ]
    )

    # Log info about configuration
    log_info = LogInfo(
        msg=['RRT Planning Orchestrator launched with Multi-Agent Planning: ', 
             LaunchConfiguration('use_multiagent')]
    )

    return LaunchDescription([
        use_multiagent_arg,
        rrt_step_size_arg,
        rrt_max_iterations_arg,
        rrt_goal_bias_arg,
        astar_time_step_arg,
        astar_collision_radius_arg,
        re_mapping_node,
        nav2_pathclient_node,
        dubins_converter_node,
        rrt_orchestrator_node,
        log_info,
    ])
