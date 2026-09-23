"""Launch one portable HRI application with a selectable YASMIN mode."""

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


MODES = {"speak", "detect_people", "move_relative"}
ROBOTS = {"nao", "pepper"}
MOVEMENT_LIMITS = (0.5, 0.3, math.pi / 2.0)


def argument(context, name):
    return LaunchConfiguration(name).perform(context)


def require_yasmin():
    missing = [name for name in ("yasmin", "yasmin_ros") if find_spec(name) is None]
    if missing:
        raise RuntimeError(
            "Missing Python modules: " + ", ".join(missing) + ". Install "
            "ros-jazzy-yasmin and ros-jazzy-yasmin-ros, then source ROS and "
            "the workspace overlay."
        )


def application_arguments(context, mode, robot):
    args = ["--mode", mode, "--robot", robot]
    if mode == "speak":
        text = argument(context, "text")
        if not text.strip():
            raise RuntimeError("Speech text cannot be empty")
        return args + ["--text", text, "--language", argument(context, "language")]

    if mode == "detect_people":
        try:
            confidence = float(argument(context, "minimum_confidence"))
            timeout = float(argument(context, "timeout"))
        except ValueError as error:
            raise RuntimeError("Confidence and timeout must be numbers") from error
        if not 0.5 <= confidence <= 1.0 or not math.isfinite(timeout) or timeout <= 0:
            raise RuntimeError("Use confidence in [0.5, 1.0] and timeout > 0")
        return args + [
            "--minimum-confidence", str(confidence),
            "--timeout", str(timeout),
        ]

    if argument(context, "confirm") != "MOVER":
        raise RuntimeError('Movement requires confirm:=MOVER')
    try:
        values = tuple(float(argument(context, name)) for name in ("x", "y", "theta"))
    except ValueError as error:
        raise RuntimeError("x, y and theta must be numbers") from error
    if not all(math.isfinite(value) for value in values):
        raise RuntimeError("x, y and theta must be finite")
    if all(abs(value) <= 1e-6 for value in values):
        raise RuntimeError("At least one movement component must be non-zero")
    if any(abs(value) > limit for value, limit in zip(values, MOVEMENT_LIMITS)):
        raise RuntimeError("Movement exceeds x=0.5 m, y=0.3 m or theta=pi/2 rad")
    return args + [
        "--x", str(values[0]), "--y", str(values[1]),
        "--theta", str(values[2]), "--confirm", "MOVER",
    ]


def launch_setup(context):
    require_yasmin()
    mode = argument(context, "mode").strip().lower()
    robot = argument(context, "robot").strip().lower()
    if mode not in MODES:
        raise RuntimeError("mode must be speak, detect_people or move_relative")
    if robot not in ROBOTS:
        raise RuntimeError("robot must be nao or pepper")
    app_args = application_arguments(context, mode, robot)

    share_parents = {
        str(Path(get_package_share_directory(package)).parent)
        for package in (
            "hri_capability_interfaces",
            "hri_naoqi_providers",
            "hri_yolo_providers",
        )
    }
    remappings = [] if robot == "nao" else [
        ("/nao/say", "/pepper/say"),
        ("/nao/move_to", "/pepper/move_to"),
        ("/nao/odom", "/pepper/odom"),
    ]
    capabilities_server = Node(
        package="capabilities2_server",
        executable="capabilities2_server_node",
        name="capabilities",
        output="screen",
        parameters=[{
            "db_file": "/tmp/portable_hri_app.sqlite3",
            "rebuild": True,
            "package_paths": sorted(share_parents),
        }],
        remappings=remappings,
    )
    application = Node(
        package="hri_reference_app",
        executable="hri_app",
        name="portable_hri_app",
        output="screen",
        arguments=app_args,
    )
    stop_when_application_finishes = RegisterEventHandler(
        OnProcessExit(
            target_action=application,
            on_exit=[EmitEvent(event=Shutdown(reason="Portable HRI application finished"))],
        )
    )
    return [capabilities_server, application, stop_when_application_finishes]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot", description="nao or pepper"),
        DeclareLaunchArgument("mode", description="speak, detect_people or move_relative"),
        DeclareLaunchArgument("text", default_value="Hola, esta es una aplicación portable con YASMIN."),
        DeclareLaunchArgument("language", default_value="Spanish"),
        DeclareLaunchArgument("minimum_confidence", default_value="0.5"),
        DeclareLaunchArgument("timeout", default_value="20.0"),
        DeclareLaunchArgument("x", default_value="0.0"),
        DeclareLaunchArgument("y", default_value="0.0"),
        DeclareLaunchArgument("theta", default_value="0.0"),
        DeclareLaunchArgument("confirm", default_value=""),
        OpaqueFunction(function=launch_setup),
    ])
