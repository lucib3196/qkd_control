from setuptools import find_packages, setup

package_name = "pan_tilt_system"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="luci",
    maintainer_email="luci@todo.todo",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "pan_tilt_system = pan_tilt_system.pan_tilt_system:main",
            "camera=pan_tilt_system.camera_system:main",
            "arduino_bridge=pan_tilt_system.arduino_bridge:main"
        ],
    },
)
