from setuptools import find_packages, setup

package_name = 'data_insights'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools',
                      'matplotlib'],
    zip_safe=True,
    maintainer='christian',
    maintainer_email='cpedrigal@scu.edu',
    description='Recording, plotting, visualizing data',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            f"live_contour_plotter = {package_name}.live_contour_plotter:main",
            f"live_read_to_csv = {package_name}.live_read_to_csv:main"
        ],
    },
)
