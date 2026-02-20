import os
from setuptools import find_packages, setup
from glob import glob

package_name = 'plan_example_python'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*')),
        ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user1',
    maintainer_email='smarios@jaist.ac.jp',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'plan = plan_example_python.motion_planning_python_api_tutorial:main',
            'object_spawner = plan_example_python.object_spawner:main',
            'random_object_spawner = plan_example_python.random_object_spawner:main',
            'pick_place = plan_example_python.pick_place:main',
            'depth_camera  = plan_example_python.img_process:main',
            'planning_service  = plan_example_python.demo_planning_service:main',
            ],
        },
    )
