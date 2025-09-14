from setuptools import setup

package_name = 'robot_nodes'

setup(
    name='robot-nodes',
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='claudio',
    maintainer_email='claudio@example.com',
    description='Robot control nodes for TIAGo manipulation',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ik = robot_nodes.ik:main',
            'aruco_detector_node = robot_nodes.aruco_detector:main',
            'head_movement_action_node = robot_nodes.head_movement_action:main',
            'state_machine = robot_nodes.state_machine:main',
        ],
    },
)