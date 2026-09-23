#!/usr/bin/env python3
"""One portable YASMIN application with independently selectable capabilities."""

import argparse
import sys

from rclpy.utilities import remove_ros_args

import move_demo
import people_demo
import speak_demo


MODES = {
    "speak": speak_demo,
    "detect_people": people_demo,
    "move_relative": move_demo,
}


def main():
    ros_free_args = remove_ros_args(args=sys.argv)[1:]
    selector = argparse.ArgumentParser(
        description="Run one capability mode of the portable HRI application.",
        add_help=False,
    )
    selector.add_argument("--mode", choices=tuple(MODES), required=True)
    selected, mode_args = selector.parse_known_args(ros_free_args)
    module = MODES[selected.mode]
    return module.run(module.parse_args(mode_args))


if __name__ == "__main__":
    raise SystemExit(main())
