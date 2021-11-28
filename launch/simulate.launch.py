from collections.abc import Iterable
from datetime import timedelta
from pathlib import Path
from typing import Union

import lifecycle_msgs.msg
import xacro
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState

import launch
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution



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

    rvizconfig_launch_arg = DeclareLaunchArgument(
        name='rvizconfig', 
        default_value=PathJoinSubstitution([package_dir,'config','rviz_config.rviz']),
        description='Absolute path to rviz config file')

    # using the version of topic_tools from https://github.com/mateusz-lichota/topic_tools
    display_conemap = Node(
        package='topic_tools',
        executable='transform_node.py',
        name='transform',
        output='screen',
        parameters=[
            {'input': '/conemap'},
            {'output-topic': '/conemap_rviz'},
            {'output-type': 'visualization_msgs/MarkerArray'},
            {'expression': 'conemap_to_markerarray.convert(m)'},
            {'import': ['conemap_to_markerarray']}
        ])



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
            transition_id = lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
        ))

    map_server_emit_configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher = launch.events.matches_action(map_server_node),
            transition_id = lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        ))

    map_server_inactive_state_handler = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node = map_server_node,
            goal_state = 'inactive',
            entities = [map_server_emit_activation_event],
        ))

    map_launch_arg = DeclareLaunchArgument(
        name='map',
        default_value= f'{package_dir}/maps/building_31.yaml')



    # ===============
    # ==  RACECAR  ==
    # ===============
    racecar_description = xacro.process(
        str(Path(get_package_share_directory('racecar_description'), 'urdf/racecar.xacro'))
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {
                'robot_description': racecar_description
            }
        ]
    )


    # =================
    # ==  SIMULATOR  ==
    # =================
    simulator_node = Node(
        package='lightweight_lidar_only_simulator',
        executable='simulate',
        name='lightweight_lidar_only_simulator',
        output='screen',
        parameters=[
            f'{package_dir}/config/params.yaml'
        ])

    return LaunchDescription([
        map_launch_arg,
        rvizconfig_launch_arg,
        display_conemap,

        map_server_node,        
        map_server_inactive_state_handler,
        delayed(map_server_emit_configure_event, timedelta(seconds=2.)),
        rviz_node,
        shutdown_on_rviz_exit,

        robot_state_publisher_node,

        simulator_node,
    ])


if __name__ == '__main__':
    generate_launch_description()
