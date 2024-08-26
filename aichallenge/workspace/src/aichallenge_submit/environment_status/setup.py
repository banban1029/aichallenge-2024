from setuptools import find_packages, setup

package_name = 'environment_status'

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
    maintainer='banban',
    maintainer_email='banshiro1029@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'object_detector = environment_status.object_detector:main',
            'map_receivement = environment_status.maps.map_receivement:main',
            'delaunay_node_base = environment_status.maps.delaunay_node_base:main',
        ],
    },
)
