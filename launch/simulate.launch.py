from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node, LifecycleNode
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription, EmitEvent, ExecuteProcess, RegisterEventHandler, include_launch_description
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
import launch
import lifecycle_msgs
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
from launch.event_handlers import OnProcessStart, OnProcessExit

def generate_launch_description():
    package = get_package_share_directory('racecar_simulator')

    default_rviz_config_path = PathJoinSubstitution(
        [
            package,
            'config',
            'rviz_config.rviz'
        ])
    
    map_launch_arg = DeclareLaunchArgument(
        name='map',
        default_value= f'{package}/maps/building_31.yaml')

    rvizconfig_launch_arg = DeclareLaunchArgument(
        name='rvizconfig', 
        default_value=default_rviz_config_path,
        description='Absolute path to rviz config file')

    # RVIZ
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

    # MAP SERVER
    map_server_node = LifecycleNode(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        namespace='',
        parameters=[
            {'yaml_filename': f'{package}/maps/building_31.yaml'}
        ])

    map_server_activate_trans_event = EmitEvent(
        event = ChangeState(
            lifecycle_node_matcher = launch.events.matches_action(map_server_node),
            transition_id = lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
        ))

    map_server_configure_trans_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher = launch.events.matches_action(map_server_node),
            transition_id = lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        ))

        
    map_server_inactive_state_handler = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node = map_server_node,
            goal_state = 'inactive',
            entities = [
                LogInfo( msg = "'map_server' reached the 'INACTIVE' state, 'activating'." ),
                map_server_activate_trans_event,
            ],
        ))
    
    map_server_active_state_handler = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node = map_server_node,
            goal_state = 'active',
            entities = [
                # Log
                LogInfo( msg = "'map_server' reached the 'ACTIVE' state" ),
            ],
        ))

    delay_map_server_configure = ExecuteProcess(
        cmd=['sleep', '1'],
        on_exit=[
            map_server_configure_trans_event
        ])

    include_racecar_model_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            f'{package}/launch/racecar_model.launch.py'
        ))

    simulator_node = Node(
        package='racecar_simulator',
        executable='simulate',
        name='racecar_simulator',
        output='screen',
        parameters=[
            f'{package}/config/params.yaml'
        ])

    ld = LaunchDescription([
        map_launch_arg,
        rvizconfig_launch_arg,
    
        map_server_node,        
        map_server_inactive_state_handler,
        map_server_active_state_handler,
        delay_map_server_configure,

        rviz_node,
        shutdown_on_rviz_exit,

        include_racecar_model_launch,

        simulator_node,
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
