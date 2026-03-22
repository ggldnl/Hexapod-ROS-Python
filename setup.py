from setuptools import find_packages, setup

package_name = 'hexapod_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'hexapod',
        'numpy',
        'pyserial',
        'pyyaml'
    ],
    zip_safe=True,
    maintainer='daniel',
    maintainer_email='danielgigliotti99.dg@gmail.com',
    description='ROS2 python node wrapping hexapod controller',
    license='MIT',
    entry_points={
        'console_scripts': [
            'hexapod_controller = hexapod_controller.hexapod_controller_node:main',
        ],
    },
)
