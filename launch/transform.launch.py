from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='elevation_map_transform',
            executable='elevation_map_transform_node',
            name='elevation_map_transform',
            output='screen',
            parameters=[{
                'roi_x_min': -1.1,
                'roi_x_max':  1.1,
                'roi_y_min': -0.6,
                'roi_y_max':  0.6,
                'sampling_resolution': 0.1
            }],
        )
    ])
