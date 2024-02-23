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

    # pub_odom = Node(
    #     package='jinpao_bringup',
    #     executable='odom_publisher.py',
    #     name='odom_publisher',
    #     remappings={("/odom", "/example/odom"),("/tf", "raw_transform")},
    # )

    # imu = Node(
    #     package='plant_message',
    #     executable='carver_esp32_node.py',
    #     output='screen',
    #     respawn=True,
    #     parameters=[{'COM': '/dev/serial/by-path/pci-0000:00:14.0-usb-0:1.3:1.0'}],
    #     remappings={("/imu/imu_data", "/example/imu")}
    # )

    launch_description = LaunchDescription()
    # launch_description.add_action(plant_feedback)
    # launch_description.add_action(pub_odom)
    launch_description.add_action(visualize)
    launch_description.add_action(localize)
    # launch_description.add_action(imu)

    return launch_description

def main(args=None):
    try:
        generate_launch_description()
    except KeyboardInterrupt:
        # quit
        sys.exit()


if __name__ == "__main__":
    main()