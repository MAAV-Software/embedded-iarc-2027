"""Command-line entry point and simple human-readable report renderer."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from .connection import DeviceDiscoveryError, PX4Connection, available_serial_ports, discover_pixhawk_device
from .config import write_snapshot
from .models import SafetyContext, Status, TestResult
from .motors import MotorSafetyError, MotorTests, confirm_vehicle_safety, validate_motor_request
from .shell import PX4Shell, ShellError
from .tests import PX4Tests


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Read-only PX4 MAVLink acceptance checks")
    parser.add_argument("command", nargs="?", choices=["full", "shell", "motors", "arm"], help="test suite to run")
    parser.add_argument("shell_command", nargs=argparse.REMAINDER, help="read-only PX4 NSH command after 'shell'")
    parser.add_argument("--device", help="MAVLink serial device, e.g. /dev/tty.usbmodemXXXX")
    parser.add_argument("--baud", type=int, default=115200, help="serial baud rate (default: 115200)")
    parser.add_argument("--timeout", type=float, default=5.0, help="per-request timeout in seconds (default: 5)")
    parser.add_argument("--list-ports", action="store_true", help="list serial ports and exit")
    parser.add_argument("--show-coordinates", action="store_true", help="include GPS coordinates in the report")
    parser.add_argument("--golden-config", type=Path, help="JSON parameter snapshot to compare during full")
    parser.add_argument("--export-config", type=Path, help="write the complete live parameter snapshot as JSON")
    parser.add_argument("--require-peripherals", action="store_true", help="treat missing battery/GPS/mag/RC/ESC hardware as FAIL")
    parser.add_argument("--spin-motors", action="store_true", help="enable motor-output tests; requires additional safety confirmation")
    parser.add_argument("--confirm-props-removed", action="store_true", help="non-interactively confirm motor-test safety (for fixtures/CI)")
    parser.add_argument("--motor", type=int, action="append", default=[], help="motor number to test (repeatable, 1..12)")
    parser.add_argument("--all-motors", action="store_true", help="test every configured motor sequentially")
    parser.add_argument("--motor-power", type=float, default=0.10, help="normalized motor test output, maximum 0.15")
    parser.add_argument("--motor-duration", type=float, default=2.0, help="per-motor duration in seconds, maximum 5")
    parser.add_argument("--controller-device", help="RadioMaster USB serial device used for gvar commands")
    parser.add_argument("--arm", action="store_true", help="enable the separately safety-gated ARM test")
    return parser


def print_ports() -> None:
    ports = available_serial_ports()
    if not ports:
        print("No serial ports found.")
        return
    for port in ports:
        print(f"{port.device}\n  {port.description}\n  {port.hardware_id}")


def print_result(result: TestResult) -> None:
    print(f"\n{result.section}")
    print(f"[{result.status.value}] {result.summary}")
    for detail in result.details:
        print(f"       {detail}")


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.list_ports:
        print_ports()
        return 0
    if not args.command:
        build_parser().print_help()
        return 2
    if args.timeout <= 0 or args.baud <= 0:
        print("--timeout and --baud must be positive.", file=sys.stderr)
        return 2
    safety_context = SafetyContext(
        vehicle_secured=confirm_vehicle_safety(args.confirm_props_removed)
    )
    if args.command == "motors":
        try:
            if not args.spin_motors:
                raise MotorSafetyError("Refusing motor output: pass --spin-motors")
            validate_motor_request(
                args.spin_motors,
                safety_context.vehicle_secured,
                args.motor,
                args.all_motors,
                args.motor_power,
                args.motor_duration,
            )
        except MotorSafetyError as error:
            print(f"Motor safety check failed: {error}", file=sys.stderr)
            return 2
    if args.command == "arm":
        if not args.arm:
            print("Refusing to arm: pass --arm", file=sys.stderr)
            return 2
        if not safety_context.vehicle_secured:
            print("Arm safety check failed: props must be removed or the vehicle securely restrained", file=sys.stderr)
            return 2
        if not args.controller_device:
            print("Arm safety check failed: pass --controller-device PATH", file=sys.stderr)
            return 2
    try:
        device = args.device or discover_pixhawk_device()
    except DeviceDiscoveryError as error:
        print(f"Device discovery failed: {error}", file=sys.stderr)
        return 2

    print("MAAV PX4 TEST")
    print(f"Device: {device} at {args.baud} baud")
    try:
        with PX4Connection(device, args.baud, args.timeout) as connection:
            if args.command == "shell":
                if not args.shell_command:
                    print("A read-only PX4 command is required after 'shell'.", file=sys.stderr)
                    return 2
                connection.wait_for_heartbeat()
                shell = PX4Shell(connection)
                try:
                    print(shell.execute(" ".join(args.shell_command)))
                finally:
                    shell.close()
                return 0
            if args.command == "motors":
                connection.wait_for_heartbeat()
                shell = PX4Shell(connection)
                try:
                    results = MotorTests(shell, safety_context).run(
                        args.motor, args.all_motors, args.motor_power, args.motor_duration
                    )
                finally:
                    shell.close()
                for result in results:
                    print_result(result)
                return 1 if any(result.status is Status.FAIL for result in results) else 0
            tests = PX4Tests(
                connection,
                args.show_coordinates,
                args.golden_config,
                args.require_peripherals,
                safety_context,
                args.controller_device,
            )
            if args.command == "arm":
                connection.wait_for_heartbeat()
                result = tests.test_arm()
                print_result(result)
                return 1 if result.status is Status.FAIL else 0
            results = tests.run_full()
            if args.export_config:
                parameters = tests.parameter_snapshot or connection.request_all_parameters()
                write_snapshot(args.export_config, parameters)
                print(f"\n[PASS] Wrote {len(parameters)} parameters to {args.export_config}")
    except (OSError, ShellError, TimeoutError) as error:
        print(f"\nPX4 CONNECTION\n[FAIL] Could not open {device}: {error}")
        return 1

    for result in results:
        print_result(result)
    return 1 if any(result.status is Status.FAIL for result in results) else 0
