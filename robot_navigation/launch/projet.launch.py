from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_perception',
            namespace='perception',
            executable='follow',
            name='turtlebot'
        ),
        Node(
            package='robot_perception',
            namespace='perception',
            executable='avoid',
            name='turtlebot'
        ),
        Node(
            package='robot_navigation',
            namespace="navigation",
            executable='navigation',
            name='turtlebot',
            
        )
    ])