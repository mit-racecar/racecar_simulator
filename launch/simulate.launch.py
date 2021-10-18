import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package = get_package_share_directory('racecar_simulator')
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='map',
            default_value= f'{package}/maps/building_31.yaml'
        ),
        launch_ros.actions.Node(
            package='joy',
            executable='joy_node',
            name='joy_node'
        ),
        launch_ros.actions.Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server'
        ),
        launch_ros.actions.Node(
            package='racecar_simulator',
            executable='simulate',
            name='racecar_simulator',
            output='screen',
            parameters=[
                f'{package}/params.yaml'
            ]
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                f'{package}/launch/racecar_model.launch.py'
            )
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
