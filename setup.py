import os
from glob import glob

from setuptools import find_packages, setup

package_name = "twist_2_0_description"


def collect_share_files(relative_dir: str) -> list[tuple[str, list[str]]]:
    """Install all files under relative_dir into share/<package>/<relative_dir>."""
    base = os.path.join(os.path.dirname(__file__), relative_dir)
    if not os.path.isdir(base):
        return []
    entries: list[tuple[str, list[str]]] = []
    for root, _dirs, files in os.walk(base, followlinks=True):
        if not files:
            continue
        rel = os.path.relpath(root, base)
        if rel == ".":
            dest = os.path.join("share", package_name, relative_dir)
        else:
            dest = os.path.join("share", package_name, relative_dir, rel)
        entries.append((dest, [os.path.join(root, f) for f in files]))
    return entries


setup(
    name=package_name,
    version="2.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        *collect_share_files("urdf"),
        *collect_share_files("meshes"),
        *collect_share_files("config"),
        *collect_share_files("rviz"),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Sirena Technologies",
    maintainer_email="dev@sirena-technologies.com",
    description="Twist 2.0 quadruped simulation and visualization",
    license="Proprietary",
    entry_points={
        "console_scripts": [
            "stand_pose = twist_2_0_description.stand_pose:main",
            "walk_gait = twist_2_0_description.walk_gait:main",
            "cmd_vel_gait = twist_2_0_description.cmd_vel_gait:main",
            "teleop_keyboard = twist_2_0_description.teleop_keyboard:main",
            "teleop_ui = twist_2_0_description.teleop_ui:main",
            "gait_test = twist_2_0_description.gait_test:main",
        ],
    },
)
