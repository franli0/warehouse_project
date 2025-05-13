from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'nav2_apps'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Navigation applications using Nav2 Simple Commander API',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'move_shelf_to_ship = nav2_apps.move_shelf_to_ship:main',
            'move_shelf_to_ship_real = nav2_apps.move_shelf_to_ship_real:main',
        ],
    },
)
