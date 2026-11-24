from glob import glob

from setuptools import find_packages, setup

package_name = "mobility_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/docs", glob("docs/*.md")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="MRT Mobility",
    maintainer_email="mrt@example.com",
    description="Single launch file and per-domain configs for the mobility stack.",
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={"console_scripts": []},
)
