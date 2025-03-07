from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='trajectory_visualization',
            executable='trajectory_publisher_saver',
            name='trajectory_publisher_saver'
        ),
        Node(
            package='trajectory_visualization',
            executable='trajectory_reader_publisher',
            name='trajectory_reader_publisher',
            parameters=[{'trajectory_file': '/path/to/your/saved_trajectory.csv'}]
        )
    ])
