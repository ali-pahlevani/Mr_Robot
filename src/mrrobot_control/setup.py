import os
from glob import glob

from setuptools import setup

package_name = "mrrobot_control"

setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "config", "missions"),
         glob("config/missions/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    author="Ali Pahlevani",
    author_email="a.pahlevani1998@gmail.com",
    maintainer="Ali Pahlevani",
    maintainer_email="a.pahlevani1998@gmail.com",
    url="https://github.com/ali-pahlevani/Mr_Robot",
    description="Controllers, arm kinematics and behaviours for mrRobot.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "posture = mrrobot_control.posture_node:main",
            "mission = mrrobot_control.mission_node:main",
            "spawn_controllers = mrrobot_control.spawn_controllers:main",
        ],
    },
)
