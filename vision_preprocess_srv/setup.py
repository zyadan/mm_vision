from setuptools import setup
import os
from glob import glob

package_name = 'vision_preprocess_srv'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zyadan',
    maintainer_email='yadan001@e.ntu.edu.sg',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vision_prepro_srv_octomap = vision_preprocess_srv_octomap.vision_prepro_srv:main',
            'vision_prepro_srv_dismap = vision_preprocess_srv_dismap.vision_prepro_srv:main',
            'vision_prepro_cli_control = vision_preprocess_srv.vision_prepro_cli_control:main',
            'vision_prepro_cli_task = vision_preprocess_srv.vision_prepro_cli_task:main',
            'vision_prepro_cli_octo = vision_preprocess_srv.vision_prepro_cli_octo:main',
            'vision_prepro_order_srv = vision_preprocess_srv.vision_prepro_order_srv:main',
            'tf2_listener = vision_preprocess_srv.tf2_listener:main',
        ],
    },
)


