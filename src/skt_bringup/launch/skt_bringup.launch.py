from ament_index_python.packages import get_package_share_path
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
import os
import sys

def generate_launch_description():
    description_launch_file_dir = os.path.join(
        get_package_share_directory("skt_description"), "launch"
    )
    
    localize_launch_file_dir = os.path.join(
        get_package_share_directory("robot_localization"), "launch"
    )

    localize = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [localize_launch_file_dir, "/ekf.launch.py"]
        ),
    )

    visualize = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [description_launch_file_dir, "/description.launch.py"]
        ),
    )

    bridge = Node(
        package='skt_bringup',
        executable='bridge_node.py',
        name='odom_publisher',
        remappings={("/odom", "/example/odom"),("/tf", "raw_transform")},
    )

    odom_republisher = Node(
        package='skt_bringup',
        executable='ekf_odom_republisher_node.py',
        name='ekf_odom_republisher_node',
    )

    launch_description = LaunchDescription()
    launch_description.add_action(bridge)
    launch_description.add_action(visualize)
    launch_description.add_action(localize)
    launch_description.add_action(odom_republisher)
    return launch_description

def main(args=None):
    try:
        generate_launch_description()
    except KeyboardInterrupt:
        # quit
        sys.exit()


if __name__ == "__main__":
    main()