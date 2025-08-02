from setuptools import find_packages, setup

package_name = 'robot_launcher'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name + '/launch', ['launch/master_launch.py']),
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
            'macchina_a_stati = robot_launcher.macchina_a_stati:main',
            'testa_e_aruco = robot_launcher.testa_e_aruco:main',
        ],
    },
)