from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='fusion_localization',
            executable='time_align',
            name='time_align',
            output='screen',
            parameters=[
                {
                    'groundtruth': '/home/lyx/fusion_localization_ws/src/fusion_localization/result/fusion_pose.txt',
                    'before_data': '/home/lyx/fusion_localization_ws/src/fusion_localization/result/gps_pose.txt',
                    'after_data': '/home/lyx/fusion_localization_ws/src/fusion_localization/result/time_align_pose.txt'
                }
            ]
        )
    ])