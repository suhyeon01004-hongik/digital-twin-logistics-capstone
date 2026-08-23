from glob import glob
from setuptools import find_packages, setup


package_name = "platform_loading_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "README.md"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=[
        "setuptools",
        "numpy==1.26.4",
        "pyserial==3.5",
        "pyzbar==0.1.9",
        "ultralytics==8.4.67",
    ],
    zip_safe=True,
    maintainer="Suhyeon Lee",
    maintainer_email="suhyeon01004-hongik@users.noreply.github.com",
    description="Camera perception and platform loading manager for refuge.",
    license="Proprietary",
    entry_points={
        "console_scripts": [
            "parcel_perception = platform_loading_control.parcel_perception_node:main",
            "platform_load_manager = platform_loading_control.platform_load_manager:main",
            "debug_image_viewer = platform_loading_control.debug_image_viewer:main",
        ],
    },
)
