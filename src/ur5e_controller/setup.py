from setuptools import find_packages, setup

package_name = 'ur5e_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/cube_pointer.launch.py']),
        ('share/' + package_name + '/config', ['config/fastdds_tablet.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='laptop',
    maintainer_email='kacper.fendrykowski@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'home_position = ur5e_controller.home_position:main',
            'cube_pointer = ur5e_controller.cube_pointer_node:main',
        ],
    },
)
