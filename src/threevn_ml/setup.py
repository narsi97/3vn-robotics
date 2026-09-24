from setuptools import find_packages, setup

package_name = 'threevn_ml'

setup(
    name=package_name,
    version='0.14.0',
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
    description='Learned target regression for the 3VN robot.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'train = threevn_ml.train:main',
        ],
    },
)
