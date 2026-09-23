"""Launch Capabilities2 and the portable YASMIN people-monitoring application."""

from importlib.util import find_spec
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


SUPPORTED_ROBOTS = {"nao", "pepper"}
YASMIN_PACKAGES = ("yasmin", "yasmin_ros")


def require_yasmin(resolver=find_spec):
    """Stop before launching ROS nodes when YASMIN is not installed."""
    missing = [package for package in YASMIN_PACKAGES if resolver(package) is None]
    if missing:
        names = ", ".join(missing)
        raise RuntimeError(
            f"Missing Python modules: {names}. Install ros-jazzy-yasmin and "
            "ros-jazzy-yasmin-ros, then source /opt/ros/jazzy/setup.bash "
            "before the workspace overlay."
        )


def launch_setup(context):
    require_yasmin()
    robot = LaunchConfiguration("robot").perform(context).strip().lower()
    if robot not in SUPPORTED_ROBOTS:
        supported = ", ".join(sorted(SUPPORTED_ROBOTS))
        raise RuntimeError(f"Unsupported robot '{robot}'. Expected one of: {supported}")

    share_parents = {
        str(Path(get_package_share_directory(package)).parent)
        for package in ("hri_capability_interfaces", "hri_yolo_providers")
    }
    capabilities_server = Node(
        package="capabilities2_server",
        executable="capabilities2_server_node",
        name="capabilities",
        output="screen",
        parameters=[
            {
                "db_file": "/tmp/portable_hri_people.sqlite3",
                "rebuild": True,
                "package_paths": sorted(share_parents),
            }
        ],
    )
    application = Node(
        package="hri_reference_app",
        executable="people_demo",
        name="portable_hri_people_app",
        output="screen",
        arguments=[
            "--robot",
            robot,
            "--minimum-confidence",
            LaunchConfiguration("minimum_confidence"),
            "--timeout",
            LaunchConfiguration("timeout"),
        ],
    )
    stop_when_application_finishes = RegisterEventHandler(
        OnProcessExit(
            target_action=application,
            on_exit=[
                EmitEvent(
                    event=Shutdown(reason="Portable HRI people application finished")
                )
            ],
        )
    )
    return [capabilities_server, application, stop_when_application_finishes]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot",
                description="Robot platform: nao or pepper.",
            ),
            DeclareLaunchArgument("minimum_confidence", default_value="0.5"),
            DeclareLaunchArgument("timeout", default_value="20.0"),
            OpaqueFunction(function=launch_setup),
        ]
    )
