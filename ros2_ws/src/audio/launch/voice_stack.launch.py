from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='audio',
            executable='wake_listener',
            name='wake_word_node',
            output='screen'
        ),
        Node(
            package='audio',
            executable='voice_session_manager',
            name='voice_session_manager',
            output='screen'
        ),
        Node(
            package='audio',
            executable='stt_whisper_node',
            name='stt_whisper',
            output='screen'
        )
    ])
