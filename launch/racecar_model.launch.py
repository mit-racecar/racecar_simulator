from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, PathJoinSubstitution, FindExecutable
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package = get_package_share_directory('racecar_simulator')
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([
            package,
            "racecar.xacro",
        ])
    ])
    ld = LaunchDescription([
        DeclareLaunchArgument(
            name='racecar_xacro',
            default_value=f'{package}/racecar.xacro'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[
                {
                    'robot_description': robot_description_content
                }
            ]
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
