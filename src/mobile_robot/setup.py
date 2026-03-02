from setuptools import find_packages, setup
import os

package_name = 'mobile_robot'


def package_files(directory: str):
    paths = []
    if not os.path.isdir(directory):
        return []

    for root, _, files in os.walk(directory):
        for f in files:
            paths.append(os.path.join(root, f))

    data_files = []
    for filepath in paths:
        install_dir = os.path.join('share', package_name, os.path.dirname(filepath))
        data_files.append((install_dir, [filepath]))

    return data_files


# ---- Build data_files explicitly (robust for colcon dry-run) ----
data_files = [
    ('share/ament_index/resource_index/packages',
     [os.path.join('resource', package_name)]),
    (os.path.join('share', package_name),
     ['package.xml']),
]
data_files += package_files('launch')
data_files += package_files('model')
data_files += package_files('parameters')
data_files += package_files('world')


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='russell',
    maintainer_email='rlgraf@ucdavis.edu',
    description='mobile_robot',
    license='TODO',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'test_publisher = mobile_robot.test:main',
            'simple_avoid_obstacle = mobile_robot.simple_avoid_obstacle:main',
            'orbit_anna = mobile_robot.orbit_anna:main',

            # People behavior scripts
            'identify5 = mobile_robot.people_behavior.identify5:main',
            'move5 = mobile_robot.people_behavior.move5:main',
            'avoid_while_interact = mobile_robot.people_behavior.avoid_while_interact:main',

            # Supervisor
            'supervisor_node = mobile_robot.supervisor.supervisor_node:main',

            # Arena generation (not a ROS node, but convenient to have as a script)
            'generate_arena2 = mobile_robot.setup:main',

            # battery behavior scripts (optional)
            'auto_recharge_node = mobile_robot.battery_behavior.auto_recharge_node:main',
            'battery_node = mobile_robot.battery_behavior.battery_node:main',
            'soft_obstacle_avoidance_node = mobile_robot.battery_behavior.soft_obstacle_avoidance_node:main'       
             ],
    },
)