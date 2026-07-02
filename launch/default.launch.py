from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ld = LaunchDescription()

    ns = '/auto'

    package_dir = FindPackageShare('geofencing')

    ld.add_action(IncludeLaunchDescription(
        PathJoinSubstitution([package_dir, 'launch', 'default.launch.py'])))

    package_dir = FindPackageShare('json_agri_format_parser')

    ld.add_action(IncludeLaunchDescription(
        PathJoinSubstitution([package_dir, 'launch', 'default.launch.py'])))

    package_dir = FindPackageShare('nav_cylinders')

    ld.add_action(IncludeLaunchDescription(
        PathJoinSubstitution([package_dir, 'launch', 'default.launch.py'])))

    package_dir = FindPackageShare('nav_line_matcher')

    ld.add_action(IncludeLaunchDescription(
        PathJoinSubstitution([package_dir, 'launch', 'default.launch.py'])))

    package_dir = FindPackageShare('nav_path_matcher')

    ld.add_action(IncludeLaunchDescription(
        PathJoinSubstitution([package_dir, 'launch', 'default.launch.py'])))

    package_dir = FindPackageShare('nav_path_recorder')

    ld.add_action(IncludeLaunchDescription(
        PathJoinSubstitution([package_dir, 'launch', 'default.launch.py'])))

    package_dir = FindPackageShare('nav_replay')

    ld.add_action(IncludeLaunchDescription(
        PathJoinSubstitution([package_dir, 'launch', 'default.launch.py'])))

    package_dir = FindPackageShare('nav_turn')

    ld.add_action(IncludeLaunchDescription(
        PathJoinSubstitution([package_dir, 'launch', 'default.launch.py'])))

    package_dir = FindPackageShare('nav_arbitration')

    yaml_path = PathJoinSubstitution(
        [package_dir, 'config', 'default.yaml'])

    ld.add_action(Node(
        package='nav_arbitration',
        namespace=ns,
        executable='nav_arbitration',
        name='arbitration',
        parameters=[yaml_path],
        output='screen'
    ))

    return ld
