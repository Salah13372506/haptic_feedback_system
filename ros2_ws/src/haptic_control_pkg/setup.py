from setuptools import setup
import os
from glob import glob

package_name = 'haptic_control_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'msg'), glob('msg/*.msg')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hamizi',
    maintainer_email='salahha1337@gmail.com',
    description='Haptic feedback control package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'haptic_publisher = haptic_control_pkg.haptic_publisher:main',
            'haptic_bridge = haptic_control_pkg.wifi_bridge:main',
        ],
    },
)