from setuptools import find_packages, setup

package_name = 'threevn_control'

setup(
    name=package_name,
    version='0.16.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    # colcon selects its Python test runner by checking for a pytest test
    # dependency HERE (PytestPythonTestingStep.match -> has_test_dependency).
    # Without it colcon silently falls back to `python3 -m unittest`, which
    # collects nothing and exits 5 -- reported as a failing package even
    # though every test passes under pytest.
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='3VN Systems',
    maintainer_email='61094479+narsi97@users.noreply.github.com',
    description='3VN robot abstraction and scenario library.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'run_scenario = threevn_control.run_scenario:main',
            'acceptance = threevn_control.acceptance:main',
        ],
    },
)
