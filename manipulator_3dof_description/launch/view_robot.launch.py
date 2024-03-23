import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro

def generate_launch_description():
    pkg_name = 'manipulator_3dof_description'

    enable_joint_state_publisher_gui = LaunchConfiguration(
        variable_name='enable_joint_state_publisher_gui'
    )

    enable_joint_state_publisher_gui_arg = DeclareLaunchArgument(
        name='enable_joint_state_publisher_gui',
        default_value='true',
        description='True to launch the joint state publisher gui, false otherwise',
        choices=['true', 'false']
    )


    rsp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                FindPackageShare(pkg_name),
                '/launch',
                '/rsp.launch.py'
            ]
        ), launch_arguments={'use_sim_time': 'true'}.items()
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        condition=IfCondition(enable_joint_state_publisher_gui)
    )

    rviz_config = os.path.join(
      get_package_share_directory(pkg_name),
      'config',
      'view_robot.rviz'
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=['-d', rviz_config]
    )

    return LaunchDescription([
        enable_joint_state_publisher_gui_arg,
        rsp_launch,
        joint_state_publisher_gui_node,
        rviz_node,
    ])