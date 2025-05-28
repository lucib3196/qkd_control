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
        ),
        Node(
            output='screen',
            emulate_tty=True,
            package='fastapi_backend',
            executable='backend',
            name="backend"
        ),
        Node(
            package='pan_tilt_system',
            emulate_tty=True,
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
            package="halfwave_plate",
            emulate_tty=True,
            executable='halfwave_plate',
            name='hw_system',
            output='screen',
        ),
        Node(
            package="halfwave_plate",
            emulate_tty=True,
            executable='halfwave_pub',
            name='pub_hw',
            output='screen',
        ),
        Node (
            package = "external_devices",
            emulate_tty = True,
            executable = 'arduino_bridge_v2',
            name = 'laser',
            output = 'screen',
        )
        
    ])