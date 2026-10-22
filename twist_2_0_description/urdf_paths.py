"""Resolve package:// mesh paths for tools that do not use the ROS package index (e.g. Gazebo)."""

from pathlib import Path


def package_uris_to_files(urdf_xml: str, package_name: str, share_dir: str) -> str:
    share = Path(share_dir).resolve().as_posix()
    prefix = f"package://{package_name}/"
    return urdf_xml.replace(prefix, f"file://{share}/")
