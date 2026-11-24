from glob import glob

from setuptools import find_packages, setup

package_name = "mobility_ros"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="MRT Mobility",
    maintainer_email="mrt@example.com",
    description="Domain-independent mobility nodes: teleop, swerve kinematics, 8-loop wheel controller, wheel odometry.",
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={"console_scripts": [
        "teleop_joy = mobility_ros.teleop_joy:main",
        "teleop_keyboard = mobility_ros.teleop_keyboard:main",
        "teleop_key_window = mobility_ros.teleop_key_window:main",
        "swerve_kinematics = mobility_ros.swerve_kinematics_node:main",
        "wheel_controller = mobility_ros.wheel_controller_node:main",
        "wheel_odometry = mobility_ros.wheel_odometry_node:main",
        "scripted_drive = mobility_ros.scripted_drive:main",
    ]},
)
