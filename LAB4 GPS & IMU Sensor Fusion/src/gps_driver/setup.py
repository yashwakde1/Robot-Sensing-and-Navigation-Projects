from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'gps_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name,'launch/'),
        glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share',package_name,'msg/'),
        glob('msg/*.msg')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dhyey',
    maintainer_email='mistry.dhy@northeastern.edu',
    description='GPS_driver',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'driver=gps_driver.driver:main',
        	'gps_sub=gps_driver.gps_subscriber:main'
        ],
    },
)
