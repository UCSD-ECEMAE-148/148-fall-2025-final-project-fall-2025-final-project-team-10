from setuptools import setup
import os
from glob import glob

package_name = 'camera_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
	(os.path.join('share', package_name, 'blobs'), glob('blobs/*.blob')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='djnighti@ucsd.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'camera_publisher = camera_pkg.camera_publisher:main',
		'processor = camera_pkg.processor:main',
		'combined = camera_pkg.combined:main',
		'camtest = camera_pkg.combined:main',
		'5fps = camera_pkg.5fps:main',
		'depth = camera_pkg.depth:main',
        ],
    },
)
