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
    def add_connection_options(target: argparse.ArgumentParser, defaults: bool) -> None:
        default = None if defaults else argparse.SUPPRESS
        target.add_argument("--device", default=default, help="Pixhawk MAVLink serial device")
        target.add_argument(
            "--baud", type=int, default=115200 if defaults else argparse.SUPPRESS,
            help="MAVLink baud rate (default: 115200)",
        )
        target.add_argument(
            "--timeout", type=float, default=5.0 if defaults else argparse.SUPPRESS,
            help="request timeout in seconds (default: 5)",
        )
        target.add_argument(
            "--list-ports", action="store_true", default=False if defaults else argparse.SUPPRESS,
            help="list serial ports and exit",
        )

    parser = argparse.ArgumentParser(description="MAAV PX4 checks")
    add_connection_options(parser, defaults=True)
    commands = parser.add_subparsers(dest="command", metavar="COMMAND")

    full = commands.add_parser("full", help="run read-only PX4 checks")
    add_connection_options(full, defaults=False)
    full.add_argument("--show-coordinates", action="store_true", help="include GPS coordinates")
    full.add_argument("--golden-config", type=Path, help="compare parameters with this JSON snapshot")
    full.add_argument("--export-config", type=Path, help="write the live parameter snapshot to JSON")
    full.add_argument("--require-peripherals", action="store_true", help="treat missing optional hardware as failures")

    shell = commands.add_parser("shell", help="run an allowed read-only PX4 NSH command")
    add_connection_options(shell, defaults=False)
    shell.add_argument("shell_command", nargs=argparse.REMAINDER, help="PX4 NSH command")

    motors = commands.add_parser("motors", help="test motors with an interactive safety confirmation")
    add_connection_options(motors, defaults=False)
    motors.add_argument("--motor", type=int, action="append", default=[], help="motor number (repeatable, 1..12)")
    motors.add_argument("--all", dest="all_motors", action="store_true", help="test every configured motor")
    motors.add_argument("--power", type=float, default=0.10, help="normalized output, maximum 0.15")
    motors.add_argument("--duration", type=float, default=2.0, help="seconds per motor, maximum 5")

    arm = commands.add_parser("arm", help="test the RadioMaster ARM switch")
    add_connection_options(arm, defaults=False)
    arm.add_argument("controller_device", help="RadioMaster USB serial device")
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
    safety_context: SafetyContext | None = None
    if args.command == "motors":
        try:
            safety_context = SafetyContext(vehicle_secured=confirm_vehicle_safety())
            validate_motor_request(
                safety_context.vehicle_secured,
                args.motor,
                args.all_motors,
                args.power,
                args.duration,
            )
        except MotorSafetyError as error:
            print(f"Motor safety check failed: {error}", file=sys.stderr)
            return 2
    if args.command == "arm":
        safety_context = SafetyContext(vehicle_secured=confirm_vehicle_safety())
        if not safety_context.vehicle_secured:
            print("Arm safety check failed: props must be removed or the vehicle securely restrained", file=sys.stderr)
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
                        args.motor, args.all_motors, args.power, args.duration
                    )
                finally:
                    shell.close()
                for result in results:
                    print_result(result)
                return 1 if any(result.status is Status.FAIL for result in results) else 0
            tests = PX4Tests(
                connection,
                getattr(args, "show_coordinates", False),
                getattr(args, "golden_config", None),
                getattr(args, "require_peripherals", False),
                safety_context,
                getattr(args, "controller_device", None),
            )
            if args.command == "arm":
                connection.wait_for_heartbeat()
                result = tests.test_arm()
                print_result(result)
                return 1 if result.status is Status.FAIL else 0
            results = tests.run_full()
            if getattr(args, "export_config", None):
                parameters = tests.parameter_snapshot or connection.request_all_parameters()
                write_snapshot(args.export_config, parameters)
                print(f"\n[PASS] Wrote {len(parameters)} parameters to {args.export_config}")
    except (OSError, ShellError, TimeoutError) as error:
        print(f"\nPX4 CONNECTION\n[FAIL] Could not open {device}: {error}")
        return 1

    for result in results:
        print_result(result)
    return 1 if any(result.status is Status.FAIL for result in results) else 0
