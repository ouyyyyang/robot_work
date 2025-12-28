from glob import glob
import os

from setuptools import setup


package_name = "patrol_control"


def pkg_path(*parts: str) -> str:
    return os.path.join(os.path.dirname(__file__), *parts)


setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="oyzh",
    maintainer_email="",
    description="Control nodes for patrol robot.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "obstacle_controller = patrol_control.obstacle_controller:main",
            "odom_tf_broadcaster = patrol_control.odom_tf_broadcaster:main",
            "patrol_manager = patrol_control.patrol_manager:main",
            "nav2_patrol_manager = patrol_control.nav2_patrol_manager:main",
            "scan_to_range = patrol_control.scan_to_range:main",
            "twist_relay = patrol_control.twist_relay:main",
            "vision_checker = patrol_control.vision_checker:main",
            "environment_markers = patrol_control.environment_markers:main",
            "wheel_joint_state_publisher = patrol_control.wheel_joint_state_publisher:main",
        ],
    },
)
