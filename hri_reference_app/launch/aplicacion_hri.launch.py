"""Integrated HRI entry: local workers + YASMIN behavior + Capabilities2 adapters."""

import importlib.util
import json
import os
from pathlib import Path
import re

import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.logging import launch_config
from launch_ros.actions import Node


APPLICATION_KEYS = {
    "image_width", "image_height", "expected_frame", "minimum_confidence", "target_height_ratio",
    "max_forward_speed", "people_provider", "motion_provider", "speak_provider", "speak_language",
    "startup_text", "speech_provider", "speech_language", "audio_input_format", "audio_topic",
    "audio_info_topic", "detections_topic", "backend_remappings",
}


def argument(context, name):
    return LaunchConfiguration(name).perform(context)


def boolean(context, name):
    value = argument(context, name).lower().strip()
    if value not in {"true", "false"}:
        raise RuntimeError(name + " must be true or false")
    return value == "true"


def workspace_directory():
    prefix = Path(get_package_prefix("hri_reference_app"))
    # Support colcon's isolated and merged install layouts.
    return prefix.parent if prefix.name == "install" else prefix.parent.parent


def local_model(value, base):
    path = Path(value).expanduser()
    path = path if path.is_absolute() else base / path
    if not path.is_file() or path.suffix != ".pt":
        raise RuntimeError(f"Local model not found: {path}. Supply an existing .pt file; no automatic download.")
    return str(path.resolve())


def load_profile(path, robot):
    with Path(path).expanduser().open(encoding="utf-8") as stream:
        profile = yaml.safe_load(stream)
    if not isinstance(profile, dict) or profile.get("robot") != robot:
        raise RuntimeError("The profile must declare the selected robot")
    app = profile.get("application")
    if not isinstance(app, dict) or set(app) != APPLICATION_KEYS:
        raise RuntimeError("Profile application keys must match the nao.yaml template")
    if any(not isinstance(value, (str, int, float, list)) or isinstance(value, bool) for value in app.values()):
        raise RuntimeError("Invalid profile application value")
    for key in ("audio_topic", "audio_info_topic", "detections_topic"):
        if not isinstance(app[key], str) or not app[key].startswith("/"):
            raise RuntimeError(key + " must be an absolute ROS topic")
    return profile


def application_module(share):
    spec = importlib.util.spec_from_file_location("hri_launch_entry", share / "launch/hri_app.launch.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def launch_setup(context):
    robot = argument(context, "robot").strip()
    if not re.fullmatch(r"[a-z][a-z0-9_]*", robot):
        raise RuntimeError("robot must be a lowercase profile identifier")
    share = Path(get_package_share_directory("hri_reference_app"))
    profile_path = argument(context, "profile") or str(share / "config" / (robot + ".yaml"))
    profile = load_profile(profile_path, robot)
    options = {key: json.dumps(value) if key == "backend_remappings" else str(value)
               for key, value in profile["application"].items()}
    options.update(robot=robot, mode="follow_person", orient_camera="false")
    for name in ("enable_motion", "transcribe_speech", "print_transcript", "start_whisper_worker"):
        options[name] = str(boolean(context, name)).lower()
    for name in ("confirm", "confirm_listen", "log_level"):
        options[name] = argument(context, name)
    for name in ("target_height_ratio", "max_forward_speed"):
        if argument(context, name):
            options[name] = argument(context, name)
    if not boolean(context, "greet"):
        options["startup_text"] = ""

    # Validate every app argument/consent and construct its actions BEFORE any
    # worker starts. This also checks selected provider packages and YASMIN.
    app = application_module(share)
    app_context = LaunchContext()
    app_context.launch_configurations.update(options)
    for action in app.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(app_context)
    app.application_arguments(app_context, "follow_person", robot)
    workspace = workspace_directory()
    if options["transcribe_speech"] == "true" and options["start_whisper_worker"] == "true" and (
            options["speech_provider"] == "hri_whisper_providers/WhisperTranscribeSpeech"):
        get_package_share_directory("hri_whisper_providers")
        app_context.launch_configurations["whisper_model_path"] = local_model(
            argument(context, "whisper_model_path") or profile["whisper_model"], workspace)

    workers = []
    if boolean(context, "start_yolo"):
        if options["people_provider"] != "hri_yolo_providers/YoloDetectPeople":
            raise RuntimeError("Custom DetectPeople provider: use start_yolo:=false and its own backend")
        yolo = profile.get("yolo", {})
        required = {"model", "device", "input_image_topic", "image_reliability", "namespace", "imgsz_height", "imgsz_width"}
        if not isinstance(yolo, dict) or set(yolo) != required:
            raise RuntimeError("Profile yolo keys must match the nao.yaml template")
        if not re.fullmatch(r"[a-z][a-z0-9_]*", str(yolo["namespace"])):
            raise RuntimeError("Use a simple YOLO namespace")
        if options["detections_topic"] != f'/{yolo["namespace"]}/detections':
            raise RuntimeError("detections_topic must match the started YOLO namespace")
        if not str(yolo["input_image_topic"]).startswith("/"):
            raise RuntimeError("YOLO image topic must be absolute")
        if str(yolo["image_reliability"]) not in {"0", "1", "2"}:
            raise RuntimeError("YOLO image reliability must be 0, 1 or 2")
        for name in ("imgsz_height", "imgsz_width"):
            if not isinstance(yolo[name], int) or not 32 <= yolo[name] <= 1280 or yolo[name] % 32:
                raise RuntimeError("YOLO inference dimensions must be multiples of 32 in [32,1280]")
        yolo_options = {key: str(value) for key, value in yolo.items()}
        yolo_options["model"] = local_model(argument(context, "yolo_model") or yolo["model"], workspace)
        # Compatible CPU path: native NNPACK warnings bypass ROS log levels.
        yolo_options.update(use_tracking="False", use_3d="False", enable="True", enable_nnpack="False",
                            use_debug=str(boolean(context, "yolo_debug")),
                            log_level=options["log_level"].strip().lower())
        yolo_launch = Path(get_package_share_directory("yolo_bringup")) / "launch/yolo.launch.py"
        # Scope worker launch arguments (e.g. model/device) away from HRI config.
        workers.append(GroupAction(actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(yolo_launch)), launch_arguments=yolo_options.items())]))

    # These actions already contain resolved values and the existing supervised
    # shutdown sequence. No second server, bond owner or copy of FollowPerson.
    application_actions = app.launch_setup(app_context)
    diagnostics = []
    if boolean(context, "record_diagnostics"):
        directory = Path(argument(context, "diagnostics_dir") or (
            Path(launch_config.log_dir) / "hri_diagnostics")).expanduser().resolve()
        diagnostics = [
            LogInfo(msg=f"Diagnóstico HRI en archivo: {directory / 'summary.json'}"),
            Node(package="hri_reference_app", executable="hri_diagnostics", name="hri_diagnostics",
                 output="screen", ros_arguments=["--log-level", "warn"],
                 parameters=[{"output_dir": str(directory), "launch_pid": os.getpid(),
                              "yolo_topic": options["detections_topic"]}]),
        ]
    return [LogInfo(msg=f"HRI {robot}: perfil {profile_path}; seguimiento y transcripción. "
                        f"Movimiento={options['enable_motion']}. Ctrl+C para terminar.")] + diagnostics + workers + application_actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot", default_value="nao"),
        DeclareLaunchArgument("profile", default_value="", description="Optional absolute robot YAML path"),
        DeclareLaunchArgument("enable_motion", default_value="false"),
        DeclareLaunchArgument("confirm", default_value="", description="MOVER for physical following"),
        DeclareLaunchArgument("transcribe_speech", default_value="true"),
        DeclareLaunchArgument("confirm_listen", default_value="", description="ESCUCHAR to activate microphone"),
        DeclareLaunchArgument("print_transcript", default_value="false"),
        DeclareLaunchArgument("log_level", default_value="warn",
                              description="warn: quiet HRI/Whisper/YOLO; info: detailed diagnostics"),
        DeclareLaunchArgument("record_diagnostics", default_value="false",
                              description="Record follow timing, speech queue and CPU to files; terminal stays quiet"),
        DeclareLaunchArgument("diagnostics_dir", default_value="", description="Default: current ROS launch log directory/hri_diagnostics"),
        DeclareLaunchArgument("greet", default_value="false", description="Optional short greeting before listening"),
        DeclareLaunchArgument("start_yolo", default_value="true", description="false to reuse an external detector"),
        DeclareLaunchArgument("yolo_debug", default_value="false"),
        DeclareLaunchArgument("start_whisper_worker", default_value="true"),
        DeclareLaunchArgument("yolo_model", default_value=""),
        DeclareLaunchArgument("whisper_model_path", default_value=""),
        DeclareLaunchArgument("target_height_ratio", default_value="", description="Default from profile: 0.75 for NAO"),
        DeclareLaunchArgument("max_forward_speed", default_value="", description="Default from profile: 0.1 m/s for NAO"),
        OpaqueFunction(function=launch_setup),
    ])
