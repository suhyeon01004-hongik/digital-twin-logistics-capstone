from glob import glob
from setuptools import find_packages, setup

package_name = 'qr_scanner'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml', 'README.md']),
        (f'share/{package_name}/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'Pillow', 'pyzbar', 'qrcode[pil]'],
    zip_safe=True,
    maintainer='Suhyeon Lee',
    maintainer_email='suhyeon01004-hongik@users.noreply.github.com',
    description='ROI-based QR scanner for ROS 2.',
    license='Proprietary',
    entry_points={
        'console_scripts': [
            'qr_scanner_node = qr_scanner.qr_scanner_node:main',
            'qr_generator = qr_scanner.qr_generator:generate_logistic_qr',
        ],
    },
)
