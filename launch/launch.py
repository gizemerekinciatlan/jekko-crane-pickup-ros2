from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from pathlib import Path
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    ld = LaunchDescription()
    
    package_path = get_package_share_path("jekko")
    urdf_model_path = Path(package_path) / 'model/jekko_urdf.urdf'
    rviz_file_path = Path(package_path) / 'config/config.rviz'

    robot_model_arg = DeclareLaunchArgument(
        name="jekko_model",
        default_value=str(urdf_model_path),
        description="This is a model of Jekko XPS532 Mini-Crane."
    )

    rviz_arg = DeclareLaunchArgument(
        name="rviz_config",
        default_value=str(rviz_file_path),
        description="This is the file where RVIZ configurations are saved."
    )

    robot_description = Command(['xacro ', LaunchConfiguration('jekko_model')])

    camera_talker_node = Node(
        package="jekko",
        name="camera_talker",
        executable="camera_talker",
        output="screen",
        namespace=''
    )

    jekko_code_node = Node(
        package="jekko",
        name="jekko_code",
        executable="jekko_code",
        output="screen",
        namespace=''
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace='',
        parameters=[{'robot_description': robot_description}]
    )

    rviz_node = Node(
        package="rviz2",
        name="rviz2",
        executable="rviz2",
        output="screen",
        namespace='',
        arguments=["-d", LaunchConfiguration("rviz_config")]
    )
    
    ld.add_action(camera_talker_node)
    ld.add_action(robot_model_arg)
    ld.add_action(jekko_code_node)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(rviz_arg)

    ld.add_action(rviz_node)

    return ld