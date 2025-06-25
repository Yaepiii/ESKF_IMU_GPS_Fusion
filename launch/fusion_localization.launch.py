from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Node parameters, including those from the YAML configuration file
    fusion_localization_params = PathJoinSubstitution([
            FindPackageShare('fusion_localization'),
            'config', 'params.yaml'
    ])

    fusion_localization_node = Node(
        package='fusion_localization',
        executable='fusion_localization',
        name='fusion_localization',
        output='screen',
        parameters=[fusion_localization_params],
        # prefix='gdb -ex run --args'
    )

    # 启动RViz2
    rviz_config_path = PathJoinSubstitution([
        FindPackageShare('fusion_localization'),
        'config',
        'fusion_localization.rviz'
    ])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    return LaunchDescription([
        fusion_localization_node,
        rviz_node
    ])
