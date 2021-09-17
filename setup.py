import os
from glob import glob
from setuptools import setup

package_name = 'tunnel'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Peter Polidoro',
    author_email='peter@polidoro.io',
    maintainer='Peter Polidoro',
    maintainer_email='peter@polidoro.io',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: BSD License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Tunnel ROS interface.',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tunnel_node ='
            ' tunnel.tunnel_node:main',
        ],
    },
)
