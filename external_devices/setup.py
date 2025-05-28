from setuptools import find_packages, setup

package_name = 'external_devices'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='luci',
    maintainer_email='luci@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arduino_test = external_devices.arduino_test:main',
            'arduino_bridge = external_devices.arduino_bridge:main',
            'arduino_bridge_v2 = external_devices.arduino_bridge_v2:main'
        ],
    },
)
