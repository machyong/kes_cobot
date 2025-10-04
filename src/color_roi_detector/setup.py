from setuptools import setup
import os
from glob import glob

package_name = 'color_roi_detector'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='ROI-based color detector (String topic).',
    license='MIT',
    entry_points={
        'console_scripts': [
            'color_roi_node = color_roi_detector.color_roi_node:main',
            'check_depth = color_roi_detector.check_depth:main',
            'find_angle = color_roi_detector.find_angle:main',
            'red_depth = color_roi_detector.red_depth:main',
            'img_capture = color_roi_detector.img_capture:main',
            'get_coord = color_roi_detector.get_coord:main',
            'robot_move = color_roi_detector.robot_move:main',
            'chat_node = color_roi_detector.chat_node:main',
            'find_robot_coord = color_roi_detector.find_robot_coord:main',
        ],
    },
)
