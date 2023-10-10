from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # DeclareLaunchArgument(
        #     'target_frame', default_value='wrist_1_link',
        #     description='Target frame name.'
        # ),
        Node(
            package='vision_preprocess_srv',
            executable='vision_prepro_srv',
            name='vision_prepro_srv',
            parameters=[
                # {'target_frame': LaunchConfiguration('target_frame')}
                {'source_frame': 'camera_link'},
                {'target_frame': 'odom'}
            ]
        ),

    ])