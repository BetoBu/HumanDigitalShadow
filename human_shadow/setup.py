from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'human_shadow'


def files(pattern):
    return glob(os.path.join(pattern))


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', files('launch/*.launch.py')),
        ('share/' + package_name + '/urdf', files('urdf/*.xacro')),
        ('share/' + package_name + '/config', files('config/*.yaml')),
        ('share/' + package_name + '/worlds', files('worlds/*.sdf')),
        ('share/' + package_name + '/rviz', files('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Human Digital Shadow team',
    maintainer_email='a01735972@tec.mx',
    description='Sombra digital humana en tiempo real con Orbbec Astra Pro, '
                'MediaPipe, ROS 2 Humble y Gazebo Fortress.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pose_node = human_shadow.pose_node:main',
            'shadow_node = human_shadow.shadow_node:main',
            'demo_node = human_shadow.demo_node:main',
        ],
    },
)
