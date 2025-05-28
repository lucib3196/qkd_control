from setuptools import find_packages, setup

package_name = 'halfwave_plate'

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
            'halfwave_plate = halfwave_plate.halfwave_plate:main',
            'halfwave_pub = halfwave_plate.halwave_plate_pub:main'
        ],
    },
)
