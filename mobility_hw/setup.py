from glob import glob

from setuptools import find_packages, setup

package_name = "mobility_hw"

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
    description="Per-domain hardware adapters (mock, gazebo, unity, real CAN) for the mobility stack.",
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={"console_scripts": [
        "mock_adapter = mobility_hw.mock_adapter:main",
        "gazebo_adapter = mobility_hw.gazebo_adapter:main",
        "unity_adapter = mobility_hw.unity_adapter:main",
        "can_adapter = mobility_hw.can_adapter:main",
    ]},
)
