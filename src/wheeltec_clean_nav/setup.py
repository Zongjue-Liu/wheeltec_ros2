from glob import glob
from setuptools import find_packages, setup

package_name = 'wheeltec_clean_nav'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/maps', glob('maps/*')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
        ('share/' + package_name + '/behavior_trees', glob('behavior_trees/*.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Zongjue Liu',
    maintainer_email='2215873441@qq.com',
    description='Clean Nav2 and AMCL integration for Wheeltec.',
    license='Proprietary',
    entry_points={
        'console_scripts': [
            'route_runner = wheeltec_clean_nav.route_runner:main',
            'trajectory_recorder = wheeltec_clean_nav.trajectory_recorder:main',
            'generate_driven_map = wheeltec_clean_nav.generate_driven_map:main',
            'generate_course_map = wheeltec_clean_nav.generate_course_map:main',
            'course_markers = wheeltec_clean_nav.course_markers:main',
        ],
    },
)
