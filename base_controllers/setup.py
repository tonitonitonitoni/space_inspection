from setuptools import find_packages, setup

package_name = 'base_controllers'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'actuator_publisher = base_controllers.actuator_publisher:main',
            'odom_subscriber = base_controllers.odom_subscriber:main',
            'position_controller = base_controllers.position_controller:main',
            'elliptical = base_controllers.elliptical:main',
            'odom_pid = base_controllers.odom_pid:main',
            'pos_2 = base_controllers.pos_2:main',
            'pos_plotter = base_controllers.pos_plotter:main',
            'joint_trajectory1 = base_controllers.joint_trajectory1:main',
            'joint_trajectory2 = base_controllers.joint_trajectory2:main',
            'jt_plotter = base_controllers.jt_plotter:main',
            'lqr_v1 = base_controllers.lqr_v1:main',
        ],
    },
)
