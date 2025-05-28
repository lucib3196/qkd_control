from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pan_tilt_system',
            executable='pan_tilt_system',
            name='pt_system1',
            output='screen',
            parameters=[{
                'pan.gpio': 17,
                'pan.angle': 0,
                'pan.angle_range': [-90.0, 90.0],
                'pan.pid': [2.0, 0.0, 0.0],
                "pan.pulse_range":[0.0005,0.0024], # Seconds
                'tilt.gpio': 27,
                'tilt.angle': 0,
                'tilt.angle_range': [-90.0, 90.0],
                'tilt.pid': [2.0, 0.0, 0.0],
                "tilt.pulse_range":[0.0005,0.0024], # Seconds
                "angle_topic": "pan_tilt_angles/system1",
                "angle_topic": "error_topic/system1",
                "tilt_command": "tilt_cmd/system1",
                "pan_command":"pan_cmd/system1",
                "pan_tilt_mode":"mode/system1",
            }]
        ),
    ])
