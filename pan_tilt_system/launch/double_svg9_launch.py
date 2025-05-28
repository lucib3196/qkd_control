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
                "pan.pulse_range":[0.0005,0.0024],
                'tilt.gpio': 27,
                'tilt.angle': 0,
                'tilt.angle_range': [-90.0, 90.0],
                'tilt.pid': [2.0, 0.0, 0.0],
                "tilt.pulse_range":[0.0005,0.0024]
                
            }]
        ),
        Node(
            package='pan_tilt_system',
            executable='pan_tilt_system',
            name='pt_system2',
            output='screen',
            parameters=[{
                'pan.gpio': 22,
                'pan.angle': 0,
                'pan.angle_range': [-90.0, 90.0],
                'pan.pid': [3.0, 0.0, 0.5],
                "pan.pulse_range":[0.0005,0.0024],
                'tilt.gpio': 23,
                'tilt.angle': 0,
                'tilt.angle_range': [-90.0, 90.0],
                'tilt.pid': [3.0, 0.0, 0.5],
                "tilt.pulse_range":[0.0005,0.0024],
                
            }]
        )
    ])
