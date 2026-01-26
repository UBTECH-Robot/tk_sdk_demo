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
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='kai.yang@x-humanoid.com',
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
            'hand_control = sdk_demo.11_hand_control:main',
            'hand_status = sdk_demo.11_hand_status:main',
            
            'battery_status_demo = sdk_demo.13_1_battery_status_demo:main',
            'power_board_status_demo = sdk_demo.13_2_power_board_status_demo:main',
            'stop_key_status_demo = sdk_demo.13_3_stop_key_status_demo:main',
            'sbus_event_demo = sdk_demo.14_sbus_event_demo:main',

            'waist_control_demo = sdk_demo.5_2_1_waist_control_demo:main',
            'arm_status_demo = sdk_demo.6_1_1_arm_status_demo:main',
            'arm_control_demo = sdk_demo.6_2_1_arm_control_demo:main',
            'arm_control_demo2 = sdk_demo.6_2_2_arm_control_demo2:main',
            'arm_control_demo3 = sdk_demo.6_2_3_arm_control_demo3:main',
            'arm_control_demo4 = sdk_demo.6_2_4_arm_control_demo4:main',           
            'arm_set_zero_demo = sdk_demo.6_2_5_arm_set_zero:main',
            'leg_status_demo = sdk_demo.7_1_1_leg_status_demo:main',
            'leg_control_demo = sdk_demo.7_2_1_leg_control_demo:main',
            'asr_demo = sdk_demo.8_1_asr_demo:main',
            'point_cloud_img_demo = sdk_demo.14_1_1_save_pointcloud_images:main',
            'point_cloud_pcd_demo = sdk_demo.14_1_2_save_point_cloud_pcd:main',
            'obstacle_detect_demo = sdk_demo.14_1_3_obstacle_detector:main',
            'battery_status_monitor = sdk_demo.demo1_battery_status_monitor:main',
            'robot_audio_control = sdk_demo.demo2_robot_audio_control:main',
            'rl_cmd_control_test = sdk_demo.rl_cmd_control_test:main',
        ],
    },
)
