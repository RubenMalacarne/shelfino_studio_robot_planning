import launch
import launch_ros
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import TimerAction

def generate_launch_description():
    re_mapping_node = launch_ros.actions.Node(
        period=0.0,
        package='planning_pkg',
        executable='re_mapping',  
        name='re_mapping',
        output='screen'
    )
    nav2_pathclient_node = launch_ros.actions.Node(
        period=0.0,
        package='planning_pkg',
        executable='nav2_pathclient',  
        name='nav2_pathclient',
        output='screen'
    )
    smooth_converter_node = launch_ros.actions.Node(
        period=1.0,
        package='planning_pkg',
        executable='smooth_converter',  
        name='smooth_converter',
        output='screen'
    )
    
    prm_orchestrator_node = TimerAction(
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
        smooth_converter_node,
        prm_orchestrator_node
    ]

    return launch.LaunchDescription(nodes)

# to launch the service smooth and send the path:  
# ros2 service call /service_trigger_smoothing_path std_srvs/srv/Trigger {}
