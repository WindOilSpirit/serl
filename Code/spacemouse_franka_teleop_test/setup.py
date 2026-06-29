from setuptools import find_packages, setup

package_name = "spacemouse_franka_teleop_test"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (
            f"share/{package_name}/config",
            ["config/teleop_params.yaml"],
        ),
        (f"share/{package_name}/launch", ["launch/spacemouse_franka_teleop.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="admin123",
    maintainer_email="admin123@example.com",
    description="SpaceMouse teleop bridge for a SERL-style Franka Cartesian impedance controller.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "teleop_node = spacemouse_franka_teleop_test.teleop_node:main",
            "pose_action_server = spacemouse_franka_teleop_test.pose_action_server:main",
            "teleop_dashboard = spacemouse_franka_teleop_test.teleop_dashboard:main",
            "check_spacemouse = spacemouse_franka_teleop_test.check_spacemouse:main",
            "sim_joint_targets = spacemouse_franka_teleop_test.sim_joint_targets:main",
        ],
    },
)
