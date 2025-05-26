from setuptools import find_packages, setup

package_name = 'synchronizer'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='christian',
    maintainer_email='cpedrigal@scu.edu',
    description='Synchronizes data from multiple sources into a single topic',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            f"collect_gps_rssi = {package_name}.collect_gps_rssi:main"
        ],
    },
)
