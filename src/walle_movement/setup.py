from setuptools import setup

package_name = 'walle_movement'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alexander Lucouw',
    maintainer_email='lucouw@gmail.com',
    description='Basic listener on cmd_vel topic',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'walle_movement_listener = walle_movement.subscriber_member_function:main'
        ],
    },
)
