from setuptools import setup

package_name = 'robot_launcher'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/full_system.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='claudio',
    maintainer_email='claudio@example.com',
    description='Robot launcher package for TIAGo manipulation',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)