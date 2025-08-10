import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'robot_launcher'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch files - usa glob per trovare tutti i .py nella cartella launch
        ('share/' + package_name + '/launch', 
         glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='claudio',
    maintainer_email='claudio@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Gli executable sono definiti in robot_nodes/setup.py
            # robot_launcher contiene solo i file di launch
        ],
    },
)