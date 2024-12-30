from setuptools import find_packages, setup
import os
import glob
package_name = 'gps_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob.glob('launch/*')),
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yash',
    maintainer_email='wakde.y@northeastern.edu',
    description='gps',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'driver = gps_driver.driver:main',
        ],
    },
)