#!/usr/bin/env python3
"""Select a Qb2 frame-rate profile through the running driver's ROS service."""

import argparse
import json
import math
from pathlib import Path
import sys

import yaml

PROFILE_NAMES = ("balanced", "motion", "mapping")


def load_profiles(path):
    with Path(path).open(encoding="utf-8") as stream:
        document = yaml.safe_load(stream)
    profiles = document.get("profiles") if isinstance(document, dict) else None
    if not isinstance(profiles, dict) or set(profiles) != set(PROFILE_NAMES):
        raise ValueError("Profile file must define balanced, motion, and mapping")
    for name, profile in profiles.items():
        rate = profile.get("target_frame_rate") if isinstance(profile, dict) else None
        if isinstance(rate, bool) or not isinstance(rate, (int, float)) or not math.isfinite(rate) or rate <= 0:
            raise ValueError(f"{name}.target_frame_rate must be finite and positive")
        if set(profile) - {"target_frame_rate", "description"}:
            raise ValueError(f"Unknown setting in profile {name}; only target_frame_rate and description are supported")
    return profiles


def default_profile_file():
    from ament_index_python.packages import get_package_share_directory

    return Path(get_package_share_directory("blickfeld_qb2_ros2_driver")) / "config" / "profiles.yaml"


def parser():
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument("profile", choices=PROFILE_NAMES, nargs="?")
    result.add_argument("--list", action="store_true", help="List profiles without contacting the driver")
    result.add_argument("--profiles-file", type=Path, help="Alternative profile YAML file")
    result.add_argument("--driver-node", default="/blickfeld_qb2_driver",
                        help="Driver node path, or snapshot device prefix such as /driver/devices/device_0")
    operation = result.add_mutually_exclusive_group()
    operation.add_argument("--apply", action="store_true", help="Apply and read back the selected sensor setting")
    operation.add_argument("--dry-run", action="store_true", help="Validate only (the default)")
    return result


def make_request(rate, apply):
    from blickfeld_qb2_ros2_driver.srv import SetScanPattern

    request = SetScanPattern.Request()
    request.fields = ["target_frame_rate"]
    request.configuration.target_frame_rate_enabled = True
    request.configuration.target_frame_rate = float(rate)
    request.dry_run = not apply
    return request


def main(argv=None):
    import rclpy
    from rclpy.utilities import remove_ros_args
    from rosidl_runtime_py.convert import message_to_ordereddict
    from blickfeld_qb2_ros2_driver.srv import SetScanPattern

    argv = sys.argv if argv is None else argv
    argument_parser = parser()
    args = argument_parser.parse_args(remove_ros_args(args=argv)[1:])
    try:
        profiles = load_profiles(args.profiles_file or default_profile_file())
    except (OSError, ValueError, yaml.YAMLError) as error:
        argument_parser.error(str(error))
    if args.list:
        for name in PROFILE_NAMES:
            profile = profiles[name]
            print(f"{name:8s} {profile['target_frame_rate']:g} Hz  {profile.get('description', '')}")
        return 0
    if args.profile is None:
        argument_parser.error("Choose a profile or use --list")
    prefix = args.driver_node.rstrip("/")
    if not prefix.startswith("/") or not prefix:
        argument_parser.error("--driver-node must be an absolute ROS node/device path")
    request = make_request(profiles[args.profile]["target_frame_rate"], args.apply)
    rclpy.init(args=argv)
    node = rclpy.create_node("qb2_profile_selector")
    try:
        service = prefix + "/set_scan_pattern"
        client = node.create_client(SetScanPattern, service)
        if not client.wait_for_service(timeout_sec=5):
            print(f"Service unavailable: {service}. Check the driver namespace and ROS_DOMAIN_ID.", file=sys.stderr)
            return 1
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=20)
        if not future.done():
            print("Request timed out; read the sensor settings before retrying an apply.", file=sys.stderr)
            return 1
        response = future.result()
        result = {"profile": args.profile, "dry_run": request.dry_run, **message_to_ordereddict(response)}
        print(json.dumps(result, indent=2))
        return 0 if response.success else 1
    except (RuntimeError, KeyboardInterrupt) as error:
        print(f"Profile request interrupted: {error}. Read settings before retrying an apply.", file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
