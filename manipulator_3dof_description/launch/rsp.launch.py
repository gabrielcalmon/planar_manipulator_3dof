import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
import xacro

# Launch the robot description from xacro and the robot_state_publisher
def generate_launch_description():
    pkg_name = 'manipulator_3dof_description'
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    use_sim_time_arg = DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='Use sim time if true')    

    xacro_file = os.path.join(
        get_package_share_directory(pkg_name),
        'urdf',
        'robot.urdf.xacro'
    )

    # robot_description_urdf = xacro.process_file(xacro_file, mappings={'gazebo_sim' : 'true'}).toxml()
    robot_description_urdf = xacro.process_file(xacro_file).toxml()

    node_robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_urdf,
                         'use_sim_time': use_sim_time
                        }],
    )

    return LaunchDescription([
        use_sim_time_arg,
        node_robot_state_publisher
    ])