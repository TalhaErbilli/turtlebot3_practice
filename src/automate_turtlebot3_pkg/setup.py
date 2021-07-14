from setuptools import setup

package_name = 'automate_turtlebot3_pkg'

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
    maintainer='talha',
    maintainer_email='t.erbilli@live.nl',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'automate_turtlebot3_node = automate_turtlebot3_pkg.automate_turtlebot3_node:main',
            'drive_dist_to_wall = automate_turtlebot3_pkg.drive_dist_to_wall:main',
            'laser_node = automate_turtlebot3_pkg.laser_node:main',
            'locomotion_node = automate_turtlebot3_pkg.locomotion_node:main',
        ],
    },
)
