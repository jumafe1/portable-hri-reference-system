"""Launch Capabilities2 and the portable YASMIN Speak application."""

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


SUPPORTED_ROBOTS = {"nao": "/nao/say", "pepper": "/pepper/say"}


def launch_setup(context):
    robot = LaunchConfiguration("robot").perform(context).strip().lower()
    if robot not in SUPPORTED_ROBOTS:
        supported = ", ".join(sorted(SUPPORTED_ROBOTS))
        raise RuntimeError(f"Unsupported robot '{robot}'. Expected one of: {supported}")

    share_parents = {
        str(Path(get_package_share_directory(package)).parent)
        for package in ("hri_capability_interfaces", "hri_naoqi_providers")
    }
    backend_service = SUPPORTED_ROBOTS[robot]
    remappings = [] if robot == "nao" else [("/nao/say", backend_service)]

    capabilities_server = Node(
        package="capabilities2_server",
        executable="capabilities2_server_node",
        name="capabilities",
        output="screen",
        parameters=[
            {
                "db_file": "/tmp/portable_hri_yasmin.sqlite3",
                "rebuild": True,
                "package_paths": sorted(share_parents),
            }
        ],
        remappings=remappings,
    )
    application = Node(
        package="hri_reference_app",
        executable="speak_demo",
        name="portable_hri_speak_app",
        output="screen",
        arguments=[
            "--robot",
            robot,
            "--text",
            LaunchConfiguration("text"),
            "--language",
            LaunchConfiguration("language"),
        ],
    )
    stop_when_application_finishes = RegisterEventHandler(
        OnProcessExit(
            target_action=application,
            on_exit=[
                EmitEvent(
                    event=Shutdown(reason="Portable HRI application finished")
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
            DeclareLaunchArgument(
                "text",
                default_value="Hola, esta es una aplicación portable con YASMIN.",
            ),
            DeclareLaunchArgument("language", default_value="Spanish"),
            OpaqueFunction(function=launch_setup),
        ]
    )
