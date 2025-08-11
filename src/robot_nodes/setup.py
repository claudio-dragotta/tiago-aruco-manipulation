from setuptools import find_packages, setup

package_name = 'robot_nodes'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'head_movement_action_node = robot_nodes.head_movement_action_node:main',
            'aruco_detector_node = robot_nodes.aruco_detector_node:main',
            'ik = robot_nodes.ik:main',
            'state_machine = robot_nodes.state_machine:main',
        ],
    },

)
