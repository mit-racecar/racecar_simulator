import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package = get_package_share_directory('lightweight_lidar_only_simulator')
    ld = launch.LaunchDescription([
        launch_ros.actions.Node(
            package='lightweight_lidar_only_simulator',
            executable='distance_transform_visualizer',
            name='distance_transform_visualizer',
            output='screen',
            parameters=[
                f'{package}/params.yaml'
            ]
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
