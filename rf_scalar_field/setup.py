from setuptools import find_packages, setup

package_name = 'rf_scalar_field'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools'
        'digi-xbee'],
    zip_safe=True,
    maintainer='christian',
    maintainer_email='cpedrigal@scu.edu',
    description='Adaptive Navigation through Radio Frequency (RF) Scalar Fields',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rf_source = rf_scalar_field.rf_source:main',
            'rf_receiver = rf_scalar_field.rf_receiver:main'
        ],
    },
)
