from setuptools import setup

package_name = 'ros2_numpy'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zyadan',
    maintainer_email='zyadan@todo.todo',
    description='forked from Box-Robotics/ros2_numpy',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'ros2_numpy = ros2_numpy.__init__',
        ],
    },
)
