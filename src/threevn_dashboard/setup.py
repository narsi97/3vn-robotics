from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'threevn_dashboard'

setup(
    name=package_name,
    version='0.8.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    include_package_data=True,
    package_data={package_name: ['static/*']},
    install_requires=['setuptools'],
    # colcon picks its Python test runner from this (see threevn_control).
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='3VN Systems',
    maintainer_email='61094479+narsi97@users.noreply.github.com',
    description='Live web dashboard and health endpoints for the 3VN arm.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'dashboard = threevn_dashboard.dashboard_node:main',
        ],
    },
)
