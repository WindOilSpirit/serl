from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    controller_name = LaunchConfiguration("controller_name")
    controller_type = LaunchConfiguration("controller_type")
    controller_config = LaunchConfiguration("controller_config")
    default_controller_config = PathJoinSubstitution(
        [
            FindPackageShare("serl_franka_ros2_control"),
            "config",
            "serl_cartesian_impedance_controller.yaml",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value=""),
            DeclareLaunchArgument(
                "controller_name", default_value="serl_cartesian_impedance_controller"
            ),
            DeclareLaunchArgument(
                "controller_type",
                default_value="serl_franka_ros2_control/SerlCartesianImpedanceController",
            ),
            DeclareLaunchArgument(
                "controller_config",
                default_value=default_controller_config,
                description="SERL Cartesian impedance controller parameter YAML.",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                namespace=namespace,
                arguments=[
                    controller_name,
                    "--controller-manager-timeout",
                    "30",
                    "--controller-type",
                    controller_type,
                    "--param-file",
                    controller_config,
                ],
                output="screen",
            ),
        ]
    )
