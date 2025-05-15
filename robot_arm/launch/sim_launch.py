from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
import os

def generate_launch_description():
    ld = LaunchDescription()

    urdf_path = FindPackageShare('robot_arm')
    #urdf_path = "/home/robot_ws/install/robot_arm/share/robot_arm"
    default_model_path = PathJoinSubstitution([urdf_path, "urdf", 'robot.urdf.xacro'])
    default_rviz_config_path = PathJoinSubstitution([urdf_path, 'rviz', 'config.rviz'])

    gui_arg = DeclareLaunchArgument(name='gui', default_value='true', choices=['true', 'false'],
                                    description='Flag to enable joint_state_publisher_gui')
    ld.add_action(gui_arg)
    rviz_arg = DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                     description='Absolute path to rviz config file')
    ld.add_action(rviz_arg)

    ld.add_action(DeclareLaunchArgument(name='model', default_value=default_model_path,
                                        description='Path to robot urdf file relative to urdf_tutorial package'))
    
    # --- Add Gazebo Launch Files ---
    gazebo_ros_package_share = FindPackageShare('gazebo_ros').find('gazebo_ros')
    
    # Ensure gzserver and gzclient are launched
    gzserver_launch_file = PathJoinSubstitution([gazebo_ros_package_share, 'launch', 'gzserver.launch.py'])
    ld.add_action(IncludeLaunchDescription(gzserver_launch_file))

    gzclient_launch_file = PathJoinSubstitution([gazebo_ros_package_share, 'launch', 'gzclient.launch.py'])
    ld.add_action(IncludeLaunchDescription(gzclient_launch_file))

    robot_description_param = {'robot_description': LaunchConfiguration('model')}
    ld.add_action(DeclareLaunchArgument('robot_description', default_value=default_model_path))

    ld.add_action(DeclareLaunchArgument('robot_description', default_value=default_model_path))

    # Include Gazebo spawn_robot launch to spawn the robot in Gazebo
    spawn_robot = Node(
    package='gazebo_ros',
    executable='spawn_entity.py',
    arguments=['-entity', 'my_robot', '-file', LaunchConfiguration('model')],
    output='screen'
    )
    ld.add_action(spawn_robot)

    ld.add_action(IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('urdf_launch'), 'launch', 'display.launch.py']),
        launch_arguments={
            'urdf_package': 'robot_arm',
            'urdf_package_path': LaunchConfiguration('model'),
            'rviz_config': LaunchConfiguration('rvizconfig'),
            'jsp_gui': LaunchConfiguration('gui')}.items()
    ))
    return ld