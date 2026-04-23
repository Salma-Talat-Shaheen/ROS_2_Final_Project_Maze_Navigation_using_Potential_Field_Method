from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'maze_navigation'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        

        (os.path.join('share', package_name, 'launch'), 
         glob('launch/*.launch.py')),
        
        (os.path.join('share', package_name, 'worlds'), 
         glob('worlds/*.world')),
        
        (os.path.join('share', package_name, 'urdf'), 
         glob('urdf/*.xacro')),
        
        (os.path.join('share', package_name, 'urdf/sensors'), 
         glob('urdf/sensors/*.xacro')),

        (os.path.join('share', package_name, 'parameters'), 
         glob('parameters/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='salmashaheen',
    maintainer_email='salma@todo.todo',
    description='Maze Navigation Project',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
           'potential_field_planner = maze_navigation.potential_field_planner:main',
            'complex_planner = maze_navigation.complex_planner:main',
        ],
    },
)
