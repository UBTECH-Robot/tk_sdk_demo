from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'sdk_demo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'msg'), glob('msg/*.msg')),
    ],
    install_requires=['setuptools', 'Pillow'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='han.li@ubtrobot.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'imu_demo = sdk_demo.3_1_imu_demo:main',
            'imu_status_demo = sdk_demo.3_2_imu_status_demo:main',
            'motor_temper_demo = sdk_demo.4_motor_temper_demo:main',
            'motor_status_demo = sdk_demo.5_motor_status_demo:main',
            'head_motor_control = sdk_demo.5_head_motor_control:main',
            'waist_motor_control = sdk_demo.6_waist_motor_control:main',
            'arm_motor_control = sdk_demo.7_arm_motor_control:main',
            'leg_motor_control = sdk_demo.8_leg_motor_control:main',
            'audio_saver = sdk_demo.9_audio_saver:main',
            'audio_player = sdk_demo.9_audio_player:main',
            'audio_asr = sdk_demo.9_audio_asr:main',
            'audio_tts = sdk_demo.9_audio_tts:main',
            'depth_camera = sdk_demo.10_depth_camera:main',
            'hand_control = sdk_demo.11_hand_control:main',
            'hand_status = sdk_demo.11_hand_status:main',
            
            'battery_status_demo = sdk_demo.13_1_battery_status_demo:main',
            'power_board_status_demo = sdk_demo.13_2_power_board_status_demo:main',
            'stop_key_status_demo = sdk_demo.13_3_stop_key_status_demo:main',
            'sbus_event_demo = sdk_demo.14_sbus_event_demo:main',
            'lidar_demo = sdk_demo.17_lidar_demo:main',
            'save_point_cloud_pcd = sdk_demo.17_save_point_cloud_pcd:main',
            'save_pointcloud_images = sdk_demo.17_save_pointcloud_images:main',
        ],
    },
)
