from setuptools import setup
import os
from glob import glob

package_name = 'robot_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        (
            'share/' + package_name,
            ['package.xml'],
        ),
        # ── launch files ──────────────────────────────────────────
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.py'),
        ),
        # ── config files (yaml + rviz) ────────────────────────────
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*.yaml') + glob('config/*.rviz'),
        ),
        # ── world files ───────────────────────────────────────────
        (
            os.path.join('share', package_name, 'worlds'),
            glob('worlds/*.sdf') + glob('worlds/*.world'),
        ),
        # ── rviz configs ──────────────────────────────────────────
        (
            os.path.join('share', package_name, 'rviz'),
            glob('rviz/*.rviz'),
        ),
        # ── maps ──────────────────────────────────────────────────
        (
            os.path.join('share', package_name, 'maps'),
            glob('maps/*'),
        ),
        # ── models: turtlebot3_world root files ───────────────────
        (
            os.path.join('share', package_name, 'models', 'turtlebot3_world'),
            glob('models/turtlebot3_world/model*.sdf') +
            glob('models/turtlebot3_world/*.config'),
        ),
        # ── models: turtlebot3_world meshes ───────────────────────
        (
            os.path.join('share', package_name, 'models', 'turtlebot3_world', 'meshes'),
            glob('models/turtlebot3_world/meshes/*'),
        ),
        # Include web assets
        (os.path.join('share', package_name, 'web'),
            glob('web/*')
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hania',
    maintainer_email='hania@todo.todo',
    description='Robot bringup launch files',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'twist_to_twiststamped = robot_bringup.twist_to_twiststamped:main',
            'joystick_ws_node = robot_bringup.joystick_ws_node:main',
        ],
    },
)