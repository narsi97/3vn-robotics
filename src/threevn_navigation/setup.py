from setuptools import find_packages, setup

package_name = 'threevn_navigation'

setup(
    name=package_name,
    version='0.19.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    # See threevn_control/setup.py: without a pytest test dependency here
    # colcon falls back to unittest, collects nothing, and reports the
    # package as failing while every test passes.
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='3VN Systems',
    maintainer_email='61094479+narsi97@users.noreply.github.com',
    description='Heading estimation for the 3VN mobile base.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'fused_odom = threevn_navigation.fused_odom:main',
        ],
    },
)
