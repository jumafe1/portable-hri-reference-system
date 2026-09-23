"""Launch one supervised portable YASMIN MoveRelative request."""

from importlib.util import find_spec
import math
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


ROBOT_INTERFACES = {
    "nao": ("/nao/move_to", "/nao/odom"),
    "pepper": ("/pepper/move_to", "/pepper/odom"),
}
MOVEMENT_LIMITS = (0.5, 0.3, math.pi / 2.0)


def launch_setup(context):
    missing = [
        name for name in ("yasmin", "yasmin_ros") if find_spec(name) is None
    ]
    if missing:
        raise RuntimeError(
            "Missing Python modules: " + ", ".join(missing) + ". Install "
            "ros-jazzy-yasmin and ros-jazzy-yasmin-ros, then source the ROS "
            "and workspace setup files."
        )

    robot = LaunchConfiguration("robot").perform(context).strip().lower()
    if robot not in ROBOT_INTERFACES:
        raise RuntimeError("robot must be nao or pepper")
    if LaunchConfiguration("confirm").perform(context) != "MOVER":
        raise RuntimeError('Movement requires confirm:=MOVER')
    try:
        values = tuple(
            float(LaunchConfiguration(name).perform(context))
            for name in ("x", "y", "theta")
        )
    except ValueError as error:
        raise RuntimeError("x, y and theta must be numbers") from error
    if not all(math.isfinite(value) for value in values):
        raise RuntimeError("x, y and theta must be finite")
    if all(abs(value) <= 1e-6 for value in values):
        raise RuntimeError("At least one movement component must be non-zero")
    if any(abs(value) > limit for value, limit in zip(values, MOVEMENT_LIMITS)):
        raise RuntimeError("Movement exceeds x=0.5 m, y=0.3 m or theta=pi/2 rad")

    share_parents = {
        str(Path(get_package_share_directory(package)).parent)
        for package in ("hri_capability_interfaces", "hri_naoqi_providers")
    }
    move_service, odometry_topic = ROBOT_INTERFACES[robot]
    remappings = [] if robot == "nao" else [
        ("/nao/move_to", move_service),
        ("/nao/odom", odometry_topic),
    ]
    capabilities_server = Node(
        package="capabilities2_server",
        executable="capabilities2_server_node",
        name="capabilities",
        output="screen",
        parameters=[
            {
                "db_file": "/tmp/portable_hri_move.sqlite3",
                "rebuild": True,
                "package_paths": sorted(share_parents),
            }
        ],
        remappings=remappings,
    )
    application = Node(
        package="hri_reference_app",
        executable="move_demo",
        name="portable_hri_move_app",
        output="screen",
        arguments=[
            "--robot", robot,
            "--x", str(values[0]),
            "--y", str(values[1]),
            "--theta", str(values[2]),
            "--confirm", "MOVER",
        ],
    )
    stop_when_application_finishes = RegisterEventHandler(
        OnProcessExit(
            target_action=application,
            on_exit=[
                EmitEvent(
                    event=Shutdown(reason="Portable HRI move application finished")
                )
            ],
        )
    )
    return [capabilities_server, application, stop_when_application_finishes]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot", description="nao or pepper"),
            DeclareLaunchArgument("x", description="Forward metres"),
            DeclareLaunchArgument("y", description="Left metres"),
            DeclareLaunchArgument("theta", description="Yaw radians"),
            DeclareLaunchArgument("confirm", description='Must be "MOVER"'),
            OpaqueFunction(function=launch_setup),
        ]
    )
