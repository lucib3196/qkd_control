from launch import LaunchDescription
from launch_ros.actions import Node # type: ignore


def generate_launch_description():
    return LaunchDescription([
        Node(
            output='screen',
            emulate_tty=True,
            package='camera',
            executable='camera_pub',
            name='camera_pub',
        ),
        Node(
            output='screen',
            emulate_tty=True,
            package='camera',
            executable='aruco_detector',
            name='detection',
        )
    ])