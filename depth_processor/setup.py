from setuptools import setup

package_name = 'depth_processor'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='amel',
    maintainer_email='your_email@example.com',
    description='Depth image processor',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'processor_node = depth_processor.processor_node:main',
            '2d_node = depth_processor.2d:main',
        ],
    },
)
