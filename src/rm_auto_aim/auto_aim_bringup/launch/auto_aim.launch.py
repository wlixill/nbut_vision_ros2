import os

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, Command
from launch import LaunchDescription
from launch_ros.actions import Node

params_file = os.path.join(
    get_package_share_directory("auto_aim_bringup"), "config/default.yaml"
)
# camera_params_file = os.path.join(
#     get_package_share_directory("daheng_camera"), "config", "camera_params.yaml"
# )
# serial_driver_config = os.path.join(
#     get_package_share_directory("rm_serial_driver"), "config", "serial_driver.yaml"
# )

robot_description = Command(
    [
        "xacro ",
        os.path.join(
            get_package_share_directory("rm_gimbal_description"),
            "urdf",
            "rm_gimbal.urdf.xacro",
        ),
    ]
)

daheng_camera_node = Node(
    package="daheng_camera",
    executable="daheng_camera_node",
    output="screen",
    emulate_tty=True,
    parameters=[params_file],
    ros_arguments=["--log-level", "daheng_camera:=INFO"],
)

detector_node = Node(
    package="armor_detector",
    executable="armor_detector_node",
    output="screen",
    emulate_tty=True,
    parameters=[params_file],
    ros_arguments=["--log-level", "armor_detector:=INFO"],
)

processor_node = Node(
    package="armor_processor",
    executable="armor_processor_node",
    output="screen",
    emulate_tty=True,
    parameters=[params_file],
    ros_arguments=["--log-level", "armor_processor:=INFO"],
)

serial_driver_node = Node(
    package="rm_serial_driver",
    executable="rm_serial_driver_node",
    output="screen",
    emulate_tty=True,
    parameters=[params_file],
    ros_arguments=["--log-level", "serial_driver:=INFO"],
)

robot_state_publisher = Node(
    package="robot_state_publisher",
    executable="robot_state_publisher",
    parameters=[{"robot_description": robot_description, "publish_frequency": 1000.0}],
)


def generate_launch_description():
    return LaunchDescription(
        [
            daheng_camera_node,
            detector_node,
            processor_node,
            serial_driver_node,
            robot_state_publisher,
        ]
    )
