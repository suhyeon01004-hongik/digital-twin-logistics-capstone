from glob import glob
from setuptools import find_packages, setup

package_name = "refuge_circulation_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/scripts", glob("scripts/*.sh")),
    ],
    install_requires=["setuptools", "Flask", "Pillow", "pyserial"],
    zip_safe=True,
    maintainer="Suhyeon Lee",
    maintainer_email="suhyeon01004-hongik@users.noreply.github.com",
    description="ROS 2 bridge, planner, and web control for the refuge circulation conveyor.",
    license="Proprietary",
    entry_points={
        "console_scripts": [
            "arduino_bridge = refuge_circulation_control.arduino_bridge:main",
            "supervisor = refuge_circulation_control.supervisor:main",
            "web_control = refuge_circulation_control.web_control:main",
            "digital_twin_compare = refuge_circulation_control.digital_twin_compare:main",
        ],
    },
)
