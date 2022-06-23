import os
import launch

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription)
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = FindPackageShare(
        package='stage_ros').find('stage_ros')
    default_rviz_config_path = os.path.join(
        pkg_share, 'rviz/robot.rviz')
    # default_bt_config_path = os.path.join(pkg_share, 'bt/nav2_dist.xml')
    default_map_path = os.path.join(pkg_share, 'map/wonik_4th.yaml')
    default_urdf_model_path = os.path.join(pkg_share, 'resource/robot.urdf')
    nav2_launch_file_dir = os.path.join(
        get_package_share_directory('nav2_bringup'), 'launch')
    param_dir = os.path.join(pkg_share, 'config/nav2_teb_params.yaml')
    control_dir = os.path.join(pkg_share, 'config/ros2_control.yml')
    urdf_model = LaunchConfiguration('urdf_model')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    # bt_config_file = LaunchConfiguration('bt_config_file')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    use_rviz = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_slam = LaunchConfiguration('slam', default=False)
    use_nav = LaunchConfiguration('nav', default=False)

    start_joint_state_publisher_cmd = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher')

    start_robot_state_publisher_cmd = Node(
        condition=IfCondition(use_robot_state_pub),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': use_sim_time,
                     'robot_description': Command(['xacro ', urdf_model])}],
        arguments=[default_urdf_model_path])

    start_rviz_cmd = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],)

    st_map2odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )

    st_odom2base_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_footprint']
    )

    st_odom2base_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link']
    )

    st_base_link2scan = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.25', '0', '0.25', '0', '0', '0', 'base_link', 'scan']
    )

    slam_toolbox = Node(
        condition=IfCondition(use_slam),
        parameters=[{'use_sim_time': use_sim_time}],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
    )

    start_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [nav2_launch_file_dir, '/bringup_launch.py']),
        launch_arguments={
            'map': default_map_path,
            'use_sim_time': use_sim_time,
            'params_file': param_dir,
            'control_file': control_dir}.items(),
        condition=launch.conditions.IfCondition(use_nav)
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            name='urdf_model',
            default_value=default_urdf_model_path,
            description='Absolute path to robot urdf file'),

        DeclareLaunchArgument(
            name='rviz_config_file',
            default_value=default_rviz_config_path,
            description='Full path to the RVIZ config file to use'),

        DeclareLaunchArgument(
            name='map_yaml_file',
            default_value=default_map_path,
            description='Full path to map file to load'),

        DeclareLaunchArgument(
            name='use_robot_state_pub',
            default_value='True',
            description='Whether to start the robot state publisher'),

        DeclareLaunchArgument(
            name='use_rviz',
            default_value='True',
            description='Whether to start RVIZ'),

        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='True',
            description='Use simulation (Gazebo) clock if true'),

        # DeclareLaunchArgument(
        #     name='bt_config_file',
        #     default_value=default_bt_config_path,
        #     description='Full paht to the Behavior Tree config file to use'),

        start_joint_state_publisher_cmd,
        start_robot_state_publisher_cmd,
        start_rviz_cmd,
        st_map2odom,
        st_odom2base_link,
        st_base_link2scan,
        start_launch_cmd,
        slam_toolbox,
    ])
