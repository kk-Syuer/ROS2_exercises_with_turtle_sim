from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'turtle_square_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob(os.path.join('launch', '*.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vboxuser',
    maintainer_email='vboxuser@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        	
        	'turtle_publisher = turtle_square_py.turtle_publisher:main',
        	'move_forward =turtle_square_py.move_forward:main',
        	'turtle_subscriber = turtle_square_py.turtle_subscriber:main',
        	'wall_square_follower = turtle_square_py.wall_square_follower:main',
        	'rainbow_client = turtle_square_py.rainbow_client:main',
            'turtle_spin_service = turtle_square_py.turtle_spin_service:main',
            'patrol_command = turtle_square_py.patrol_command:main',


        ],
    },
)
