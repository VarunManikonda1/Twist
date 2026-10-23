import os
from setuptools import setup

PKG = "twist_walker"


def tree(folder):
    """Install every file under `folder`, keeping sub-directories (e.g. meshes/visual)."""
    out = []
    for root, _, files in os.walk(folder):
        if files:
            out.append((os.path.join("share", PKG, root), [os.path.join(root, f) for f in files]))
    return out


setup(
    name=PKG,
    version="0.1.0",
    packages=[PKG],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + PKG]),
        ("share/" + PKG, ["package.xml"]),
    ]
    + tree("urdf")
    + tree("meshes")
    + tree("config")
    + tree("launch"),
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Varun",
    maintainer_email="varunmanikonda0@gmail.com",
    description="Twist 2.0 quadruped simulation and gait",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "gait_node = twist_walker.gait_node:main",
            "motor_logger = twist_walker.motor_logger:main",
            "record_walk = twist_walker.record_walk:main",
        ],
    },
)
