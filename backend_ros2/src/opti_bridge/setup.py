from setuptools import find_packages, setup

package_name = "opti_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/opti_bridge.launch.py"]),
        (f"share/{package_name}/config", ["config/opti_config.yaml"]),
    ],
    install_requires=["setuptools", "PyYAML"],
    zip_safe=False,
    maintainer="you",
    maintainer_email="you@todo",
    description="Minimal OptiTrack MoCap bridge (VRPN) publishing ROS2 PoseStamped topics.",
    license="MIT",
    entry_points={"console_scripts": ["opti_bridge_node = opti_bridge.node:main"]},
)