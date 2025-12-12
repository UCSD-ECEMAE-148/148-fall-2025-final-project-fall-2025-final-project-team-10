from setuptools import setup

package_name = 'mux_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/mux_launch.py']),
        ('share/' + package_name + '/config', ['mux_pkg/twist_mux_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Twist multiplexer controller package for locked_on/is_found logic',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mux_controller = mux_pkg.mux_controller:main',
            'stop_publisher = mux_pkg.stop_publisher:main',
        ],
    },
)
