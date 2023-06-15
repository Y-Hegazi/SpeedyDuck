from setuptools import setup
import os
from glob import glob

package_name = 'duck_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
        # (os.path.join('share', package_name, 'launch'), glob('maps'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hegazi',
    maintainer_email='yousefhegazi74@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ["service_node = duck_navigation.map_client:main"
        ],
    },
)
