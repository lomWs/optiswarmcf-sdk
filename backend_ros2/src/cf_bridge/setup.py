from setuptools import find_packages, setup

package_name = "cf_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/cf_bridge.launch.py"]),
        (f"share/{package_name}/config", ["config/cf_bridge.yaml"]),
    ],
    install_requires=[
        "setuptools",
        "PyYAML",
        "cflib",
    ],
    zip_safe=True,
    maintainer="lab",
    maintainer_email="email@example.com",
    description="ROS 2 bridge between canonical mocap topics and Crazyflie radio control.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "cf_bridge_node = cf_bridge.node:main",
        ],
    },
)