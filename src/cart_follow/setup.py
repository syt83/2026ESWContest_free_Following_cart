from setuptools import find_packages, setup


package_name = 'cart_follow'


setup(
    name=package_name,

    version='0.0.0',

    packages=find_packages(
        exclude=['test']
    ),

    data_files=[
        (
            'share/ament_index/resource_index/packages',
            [
                'resource/' + package_name
            ]
        ),

        (
            'share/' + package_name,
            [
                'package.xml'
            ]
        ),
    ],

    install_requires=[
        'setuptools',
    ],

    zip_safe=True,

    maintainer='pi',

    maintainer_email='pi@example.com',

    description=(
        'UWB, LiDAR, IMU and NFC based '
        'person-following cart'
    ),

    license='Apache-2.0',

    tests_require=[
        'pytest'
    ],

    entry_points={
        'console_scripts': [

            'uwb_tracker = '
            'cart_follow.uwb_tracker:main',

            'follow_controller = '
            'cart_follow.follow_controller:main',

            'lidar_avoidance = '
            'cart_follow.lidar_avoidance:main',

            'motor_serial = '
            'cart_follow.motor_serial:main',

            'imu_node = '
            'cart_follow.imu_node:main',

            'system_manager = '
            'cart_follow.system_manager:main',

        ],
    },
)