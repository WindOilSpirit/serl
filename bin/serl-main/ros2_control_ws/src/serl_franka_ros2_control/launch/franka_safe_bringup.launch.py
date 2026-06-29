import xacro

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, Shutdown
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_robot_nodes(context):
    load_gripper_config = LaunchConfiguration("load_gripper").perform(context)
    load_gripper = load_gripper_config.lower() == "true"
    namespace = LaunchConfiguration("namespace").perform(context)

    urdf_path = PathJoinSubstitution(
        [FindPackageShare("franka_description"), "robots", LaunchConfiguration("urdf_file")]
    ).perform(context)
    robot_description = xacro.process_file(
        urdf_path,
        mappings={
            "ros2_control": "true",
            "arm_id": LaunchConfiguration("arm_id").perform(context),
            "arm_prefix": LaunchConfiguration("arm_prefix").perform(context),
            "robot_ip": LaunchConfiguration("robot_ip").perform(context),
            "hand": load_gripper_config,
            "use_fake_hardware": LaunchConfiguration("use_fake_hardware").perform(context),
            "fake_sensor_commands": LaunchConfiguration("fake_sensor_commands").perform(context),
        },
    ).toprettyxml(indent="  ")

    controllers_yaml = PathJoinSubstitution(
        [FindPackageShare("franka_bringup"), "config", "controllers.yaml"]
    ).perform(context)
    realtime_override_yaml = PathJoinSubstitution(
        [
            FindPackageShare("serl_franka_ros2_control"),
            "config",
            "franka_state_broadcaster_realtime_override.yaml",
        ]
    ).perform(context)

    joint_state_sources = ["franka/joint_states", "franka_gripper/joint_states"]
    joint_state_rate = int(LaunchConfiguration("joint_state_rate").perform(context))
    realtime_cpu = LaunchConfiguration("realtime_cpu").perform(context)
    non_realtime_cpus = LaunchConfiguration("non_realtime_cpus").perform(context)

    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace=namespace,
            parameters=[{"robot_description": robot_description}],
            prefix=f"taskset -c {non_realtime_cpus}",
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            namespace=namespace,
            parameters=[
                controllers_yaml,
                realtime_override_yaml,
                {"robot_description": robot_description},
                {"load_gripper": load_gripper},
            ],
            remappings=[("joint_states", joint_state_sources[0])],
            prefix=f"taskset -c {realtime_cpu}",
            output="screen",
            on_exit=Shutdown(),
        ),
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher",
            namespace=namespace,
            parameters=[
                {
                    "source_list": joint_state_sources,
                    "rate": joint_state_rate,
                    "use_robot_description": False,
                }
            ],
            prefix=f"taskset -c {non_realtime_cpus}",
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            namespace=namespace,
            arguments=["joint_state_broadcaster"],
            prefix=f"taskset -c {non_realtime_cpus}",
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            namespace=namespace,
            arguments=["franka_robot_state_broadcaster"],
            parameters=[{"arm_id": LaunchConfiguration("arm_id").perform(context)}],
            condition=IfCondition(LaunchConfiguration("load_franka_state_broadcaster")),
            prefix=f"taskset -c {non_realtime_cpus}",
            output="screen",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare("franka_gripper"), "launch", "gripper.launch.py"])]
            ),
            launch_arguments={
                "namespace": namespace,
                "robot_ip": LaunchConfiguration("robot_ip").perform(context),
                "use_fake_hardware": LaunchConfiguration("use_fake_hardware").perform(context),
            }.items(),
            condition=IfCondition(LaunchConfiguration("load_gripper")),
        ),
    ]


def generate_launch_description():
    launch_args = [
        DeclareLaunchArgument("arm_id", default_value="", description="ID of the type of arm used"),
        DeclareLaunchArgument("arm_prefix", default_value="", description="Prefix for arm topics"),
        DeclareLaunchArgument("namespace", default_value="", description="Namespace for the robot"),
        DeclareLaunchArgument(
            "urdf_file", default_value="fr3/fr3.urdf.xacro", description="Path to URDF file"
        ),
        DeclareLaunchArgument(
            "robot_ip", default_value="172.16.0.3", description="Hostname or IP address of the robot"
        ),
        DeclareLaunchArgument("load_gripper", default_value="false"),
        DeclareLaunchArgument("load_franka_state_broadcaster", default_value="true"),
        DeclareLaunchArgument("use_fake_hardware", default_value="false"),
        DeclareLaunchArgument("fake_sensor_commands", default_value="false"),
        DeclareLaunchArgument("joint_state_rate", default_value="30"),
        DeclareLaunchArgument("realtime_cpu", default_value="30"),
        DeclareLaunchArgument("non_realtime_cpus", default_value="0-29,31"),
    ]

    return LaunchDescription(launch_args + [OpaqueFunction(function=generate_robot_nodes)])
