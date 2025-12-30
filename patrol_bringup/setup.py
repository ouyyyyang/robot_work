from glob import glob
import os

from setuptools import setup


package_name = "patrol_bringup"


def pkg_path(*parts: str) -> str:
    return os.path.join(os.path.dirname(__file__), *parts)


setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob(pkg_path("launch", "*.launch.py"))),
        ("share/" + package_name + "/config", glob(pkg_path("config", "*.yaml"))),
        ("share/" + package_name + "/urdf", glob(pkg_path("urdf", "*.urdf"))),
        (
            "share/" + package_name + "/worlds",
            [pkg_path("..", "worlds", "patrol_world.sdf")],
        ),
        (
            "share/" + package_name + "/models/patrol_environment",
            [
                pkg_path("..", "models", "patrol_environment", "model.sdf"),
                pkg_path("..", "models", "patrol_environment", "model.config"),
            ],
        ),
        (
            "share/" + package_name + "/models/patrol_robot",
            [
                pkg_path("..", "models", "patrol_robot", "model.sdf"),
                pkg_path("..", "models", "patrol_robot", "model.config"),
            ],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="oyzh",
    maintainer_email="",
    description="Bringup (Gazebo) for the patrol robot simulation.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [],
    },
)
