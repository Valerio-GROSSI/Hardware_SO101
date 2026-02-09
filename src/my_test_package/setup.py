import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'my_test_package'

setup(
    name=package_name,
    version='0.0.0',
    # packages=find_packages(exclude=['test']),
    packages=find_packages(include=[package_name, f'{package_name}.*'], exclude=['test']), # This will find my_test_package and my_test_package.lerobot
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*_launch.py'))), # Add this line to include your launch files
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='valerio',
    maintainer_email='valerio1.grossi@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'so101_ros2_pub = my_test_package.so101_ros2_pub:main',
            # 'so101_ros2_sub = my_test_package.so101_ros2_sub:main'
        ],
    },
)
