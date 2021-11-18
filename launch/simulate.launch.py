from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node, LifecycleNode
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, EmitEvent, ExecuteProcess, RegisterEventHandler
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
import launch
import lifecycle_msgs.msg
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
from launch.event_handlers import OnProcessExit
from datetime import timedelta

def delayed(e: EmitEvent, by: timedelta) -> ExecuteProcess:
    return ExecuteProcess(
        cmd=['sleep', str(by.total_seconds())],
        on_exit=[
            e
        ]
    )

def generate_launch_description():
    package_dir = get_package_share_directory('lightweight_lidar_only_simulator')


    # ============
    # ==  RVIZ  ==
    # ============
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')])

    shutdown_on_rviz_exit = launch.actions.RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=rviz_node,
            on_exit=[
                launch.actions.EmitEvent(event=launch.events.Shutdown()),
            ],
        ))

    default_rviz_config_path = PathJoinSubstitution(
        [
            package_dir,
            'config',
            'rviz_config.rviz'
        ])

    rvizconfig_launch_arg = DeclareLaunchArgument(
        name='rvizconfig', 
        default_value=default_rviz_config_path,
        description='Absolute path to rviz config file')



    # ================ 
    # == MAP SERVER ==
    # ================
    map_server_node = LifecycleNode(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        namespace='',
        parameters=[
            {'yaml_filename': f'{package_dir}/maps/building_31.yaml'}
        ])

    map_server_emit_activation_event = EmitEvent(
        event = ChangeState(
            lifecycle_node_matcher = launch.events.matches_action(map_server_node),
            transition_id = lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE, # type: ignore
        ))

    map_server_emit_configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher = launch.events.matches_action(map_server_node),
            transition_id = lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE, # type: ignore
        ))

    map_server_inactive_state_handler = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node = map_server_node,
            goal_state = 'inactive',
            entities = [
                map_server_emit_activation_event,
            ],
        ))

    map_launch_arg = DeclareLaunchArgument(
        name='map',
        default_value= f'{package_dir}/maps/building_31.yaml')



    include_racecar_model_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            f'{package_dir}/launch/racecar_model.launch.py'
        ))

    simulator_node = Node(
        package='lightweight_lidar_only_simulator',
        executable='simulate',
        name='lightweight_lidar_only_simulator',
        output='screen',
        parameters=[
            f'{package_dir}/config/params.yaml'
        ])

    ld = LaunchDescription([
        map_launch_arg,
        rvizconfig_launch_arg,
    
        map_server_node,        
        map_server_inactive_state_handler,
        delayed(map_server_emit_configure_event, by=timedelta(seconds=1.5)),

        rviz_node,
        shutdown_on_rviz_exit,

        include_racecar_model_launch_description,

        simulator_node,
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
