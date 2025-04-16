from setuptools import setup

package_name = 'articubot_one'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),  # <== this is why __init__.py is needed
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your name',
    maintainer_email='your@email.com',
    description='Your robot package',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_to_ESP32 = articubot_one.lidar_to_ESP32:main',
            'lidar_self_driving = articubot_one.lidar_self_driving:main',
            'serial_odometry_node = articubot_one.serial_odometry_node:main',
        ],
    },
)
