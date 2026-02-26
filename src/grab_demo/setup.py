from setuptools import find_packages, setup

package_name = 'grab_demo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='han.li@ubtrobot.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_detect_node = grab_demo.yolo_detect_node:main',
            'grasp_executor_node = grab_demo.grasp_executor_node:main',
            # 保留旧节点入口，不再使用但暂不删除
            'yolo_grab_node = grab_demo.yolo_grab_node:main',
            'ik_client_node = grab_demo.ik_client_node:ros_main',
        ],
    },
)
