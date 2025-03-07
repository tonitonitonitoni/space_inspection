import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.actions import Node
#import xacro

def generate_launch_description():

    # Setup project paths
    pkg_project_bringup = get_package_share_directory('base_bringup')
    pkg_project_gazebo = get_package_share_directory('base_gazebo')
    pkg_project_description = get_package_share_directory('base_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    xacro_file = os.path.join(pkg_project_description,'urdf','test2.urdf.xacro')
    robot_description_config = Command(['xacro ', xacro_file])
    #robot_description_config = xacro.process_file(xacro_file).toxml()
    params = {'robot_description': robot_description_config}

    
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # Setup to launch the simulator and Gazebo world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': PathJoinSubstitution([
            pkg_project_gazebo,
            'worlds',
            'empty_testbed.sdf'
        ])}.items(),
    )
    
    # Spawn the robot in the Gazebo world
    spawn_entity = Node(package='ros_gz_sim', executable='create',
                        arguments=['-topic', '/robot_description',
                                   '-name', 'free_flyer',
                                   '-x', '0', 
                                   '-y', '0',
                                   '-z', '0',
                                   ],
                        output='screen')
    
    # Bridge ROS topics and Gazebo messages for establishing communication
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_project_bringup, 'config', 'bridge.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )

    # Use cmd_vel input to provide forces to actuators
    vel_publisher = Node(
        package='base_controllers',
        executable = 'odom_subscriber',
        output='screen'
    )

    return LaunchDescription([
        SetEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            PathJoinSubstitution([pkg_project_description, 'urdf'])
        ),
        node_robot_state_publisher,
        gz_sim,
        spawn_entity,
        bridge,
        vel_publisher,
    ])