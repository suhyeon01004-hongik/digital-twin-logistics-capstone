from glob import glob

from setuptools import find_packages, setup

package_name = "hik_camera"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "README.md"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="Suhyeon Lee",
    maintainer_email="suhyeon01004-hongik@users.noreply.github.com",
    description="Hikrobot MVS camera publisher for ROS 2.",
    license="Proprietary",
    entry_points={
        "console_scripts": [
            "hik_camera = hik_camera.hik_camera_node:main",
            "hik_video_recorder = hik_camera.video_recorder_node:main",
        ],
    },
)
