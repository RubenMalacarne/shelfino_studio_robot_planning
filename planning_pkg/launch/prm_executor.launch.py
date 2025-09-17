import launch
import launch_ros
from launch.actions import TimerAction

def generate_launch_description():
    re_mapping_node = launch_ros.actions.Node(
        package='planning_pkg',
        executable='re_mapping',
        name='re_mapping',
        output='screen'
    )

    nav2_pathclient_node = launch_ros.actions.Node(
        package='planning_pkg',
        executable='nav2_pathclient',
        name='nav2_pathclient',
        output='screen'
    )

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


    orchestrator_node = TimerAction(
        period=2.0,
        actions=[
            launch_ros.actions.Node(
                package='planning_pkg',
                executable='path_orchestrator_prm',
                name='path_orchestrator_prm',
                output='screen'
            )
        ]
    )

    nodes = [
        re_mapping_node,
        nav2_pathclient_node,
        dubins_converter_node, 
        orchestrator_node
    ]

    return launch.LaunchDescription(nodes)
