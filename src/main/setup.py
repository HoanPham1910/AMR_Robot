from setuptools import setup, find_packages

package_name = 'main'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(include=['Design_Frame', 'Library', 'UI']),
    py_modules=['giaodien', 'main_navi'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Hoan Pham',
    maintainer_email='hoan@example.com',
    description='Main control and GUI for AMR using ROS2 Humble',
    license='MIT',
    entry_points={
    'console_scripts': [
        'main = main.main:main.main_navi',
        'encoder_joint_state_publisher = main.Topic_python.joint_states:main',
        'cmd_vel_publisher = main.Topic_python.cmd_vel:main',
        'odom_publisher = main.Topic_python.odom:main',
        'odom_tf_broadcaster = main.Topic_python.tf_broadcaster:main',
    ],
},
)

