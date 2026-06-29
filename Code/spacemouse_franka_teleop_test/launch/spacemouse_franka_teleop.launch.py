import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    package_name = "spacemouse_franka_teleop_test"
    project_dir = os.environ.get(
        "SPACEMOUSE_FRANKA_TELEOP_DIR",
        "/home/admin123/WenshuoZhou/SERL/Code/spacemouse_franka_teleop_test",
    )
    workspace_dir = os.environ.get("SERL_WORKSPACE", "/home/admin123/WenshuoZhou/SERL")
    python_executable = os.path.join(workspace_dir, ".venv", "bin", "python")
    package_source_dir = os.path.join(project_dir)
    params_file = os.path.join(
        get_package_share_directory(package_name),
        "config",
        "teleop_params.yaml",
    )
    env = {
        "PYTHONPATH": package_source_dir + os.pathsep + os.environ.get("PYTHONPATH", ""),
    }

    return LaunchDescription(
        [
            ExecuteProcess(
                cmd=[
                    python_executable,
                    "-m",
                    "spacemouse_franka_teleop_test.teleop_node",
                    "--ros-args",
                    "-r",
                    "__node:=spacemouse_franka_teleop_test",
                    "--params-file",
                    params_file,
                ],
                output="screen",
                additional_env=env,
            ),
            ExecuteProcess(
                cmd=[
                    python_executable,
                    "-m",
                    "spacemouse_franka_teleop_test.pose_action_server",
                    "--ros-args",
                    "-r",
                    "__node:=spacemouse_franka_impedance_teleop_server",
                    "--params-file",
                    params_file,
                ],
                output="screen",
                additional_env=env,
            ),
        ]
    )
