from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'camera'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test'], include=['camera', 'camera.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
         (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma][xml]*')))

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='luci2',
    maintainer_email='luci2@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'camera_pub = camera.camera_publisher:main',
        'camera_sub = camera.camera_subscriber:main',
        'aruco_detector = camera.aruco_detection:main'
    ],
},
)
