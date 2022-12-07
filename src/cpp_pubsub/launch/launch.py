from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cpp_pubsub',
            namespace='talker',
            executable='talker',
            name='pub'
        ),
        Node(
            package='cpp_pubsub',
            namespace='listener',
            executable='listener',
            name='sub'
        ),
    ])