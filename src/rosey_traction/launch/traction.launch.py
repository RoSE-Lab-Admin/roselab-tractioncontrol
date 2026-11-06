from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_params = PathJoinSubstitution([
        FindPackageShare('rosey_traction'),
        'config',
        'traction.yaml',
    ])

    params_file = LaunchConfiguration('traction_params')

    traction_node = Node(
        package='rosey_traction',
        executable='traction_node',
        name='traction_node',
        parameters=[params_file],
        output='both',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'traction_params',
            default_value=default_params,
            description='Parameter file for traction_node'),
        traction_node,
    ])
