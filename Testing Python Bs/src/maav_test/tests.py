"""Individual read-only PX4 acceptance checks."""

from __future__ import annotations

import importlib.util
from pathlib import Path
import re
import sys
import time
from typing import Any

from pymavlink import mavutil

from .connection import PX4Connection, decode_parameter_value, git_hash_from_version
from .config import differences, read_snapshot
from .models import SafetyContext, Status, TestResult
from .shell import PX4Shell, ShellError


GPS_FIX_NAMES = {
    0: "No GPS",
    1: "No fix",
    2: "2D fix",
    3: "3D fix",
    4: "DGPS",
    5: "RTK float",
    6: "RTK fixed",
    7: "Static",
    8: "PPP",
}


def _controller_control_module() -> Any:
    """Load the shared RadioMaster gvar command definitions from this repository."""
    module_path = Path(__file__).resolve().parents[3] / "controller_auton" / "controller_control.py"
    spec = importlib.util.spec_from_file_location("maav_controller_control", module_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Could not load controller command definitions from {module_path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class PX4Tests:
    """A compact test suite that reports facts without changing vehicle state."""

    def __init__(
        self,
        connection: PX4Connection,
        show_coordinates: bool = False,
        golden_config: Path | None = None,
        require_peripherals: bool = False,
        safety_context: SafetyContext | None = None,
        controller_device: str | None = None,
    ) -> None:
        self.connection = connection
        self.show_coordinates = show_coordinates
        self.golden_config = golden_config
        self.require_peripherals = require_peripherals
        self.safety_context = safety_context
        self.controller_device = controller_device
        self.parameter_snapshot: dict[str, int | float] | None = None

    def test_connection(self) -> TestResult:
        try:
            heartbeat = self.connection.wait_for_heartbeat()
        except (OSError, TimeoutError) as error:
            return TestResult("PX4 CONNECTION", Status.FAIL, str(error))
        autopilot = mavutil.mavlink.enums["MAV_AUTOPILOT"].get(heartbeat.autopilot)
        vehicle_type = mavutil.mavlink.enums["MAV_TYPE"].get(heartbeat.type)
        return TestResult(
            "PX4 CONNECTION",
            Status.PASS,
            "Heartbeat received",
            [
                f"System ID: {self.connection.target_system}",
                f"Component ID: {self.connection.target_component}",
                f"Autopilot: {autopilot.name if autopilot else heartbeat.autopilot}",
                f"Vehicle type: {vehicle_type.name if vehicle_type else heartbeat.type}",
            ],
        )

    def test_vehicle_id(self) -> TestResult:
        message = self.connection.request_parameter("MAV_SYS_ID")
        if message is None:
            return TestResult("CONFIGURATION", Status.FAIL, "MAV_SYS_ID parameter timed out")
        value = decode_parameter_value(message)
        status = Status.PASS if value == self.connection.target_system else Status.WARN
        summary = f"MAV_SYS_ID = {value}"
        details = [] if status is Status.PASS else [f"Heartbeat system ID was {self.connection.target_system}"]
        return TestResult("CONFIGURATION", status, summary, details)

    def test_parameter_configuration(self) -> TestResult:
        if self.golden_config is None:
            return TestResult("PARAMETER CONFIGURATION", Status.SKIP, "No --golden-config supplied")
        golden = read_snapshot(self.golden_config)
        self.parameter_snapshot = self.connection.request_all_parameters()
        missing, changed = differences(golden, self.parameter_snapshot)
        if not missing and not changed:
            return TestResult(
                "PARAMETER CONFIGURATION", Status.PASS, f"Matched {len(golden)} golden parameters"
            )
        details = [f"Missing: {name}" for name in missing[:10]]
        details.extend(changed[:10])
        remaining = len(missing) + len(changed) - len(details)
        if remaining > 0:
            details.append(f"... and {remaining} more differences")
        return TestResult(
            "PARAMETER CONFIGURATION",
            Status.FAIL,
            f"{len(missing)} missing and {len(changed)} changed parameters",
            details,
        )

    def test_firmware(self) -> TestResult:
        message = self.connection.request_autopilot_version()
        if message is None:
            return TestResult("FIRMWARE", Status.FAIL, "AUTOPILOT_VERSION timed out")
        version = message.flight_sw_version
        major, minor, patch = (version >> 24) & 0xFF, (version >> 16) & 0xFF, (version >> 8) & 0xFF
        return TestResult(
            "FIRMWARE",
            Status.PASS,
            "AUTOPILOT_VERSION received",
            [
                f"PX4 version: {major}.{minor}.{patch}",
                f"Flight custom version (hex): {git_hash_from_version(message)}",
                f"Board vendor/product ID: {message.vendor_id}/{message.product_id}",
            ],
        )

    def test_battery(self) -> TestResult:
        message = self.connection.request_telemetry(
            mavutil.mavlink.MAVLINK_MSG_ID_BATTERY_STATUS, "BATTERY_STATUS"
        )
        if message is None:
            return TestResult("BATTERY", Status.FAIL, "BATTERY_STATUS timed out")
        voltages = [voltage for voltage in message.voltages if voltage != 0xFFFF]
        details = [f"Battery ID: {message.id}", f"Status: {self._battery_status_name(message)}"]
        if voltages:
            details.append(f"Voltage: {sum(voltages) / 1000:.2f} V")
        else:
            details.append("Voltage: not reported")
        if message.current_battery != -1:
            details.append(f"Current: {message.current_battery / 100:.2f} A")
        if message.battery_remaining != -1:
            details.append(f"Remaining: {message.battery_remaining}%")
        status = Status.PASS if voltages else self._missing_peripheral_status()
        summary = "Battery telemetry received" if voltages else "Battery telemetry received, but no battery voltage is available"
        return TestResult("BATTERY", status, summary, details)

    def test_gps(self) -> TestResult:
        message = self.connection.request_telemetry(
            mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT, "GPS_RAW_INT"
        )
        if message is None:
            return TestResult("GPS", self._missing_peripheral_status(), "GPS_RAW_INT not available (GPS may be disconnected)")
        fix_name = GPS_FIX_NAMES.get(message.fix_type, f"Unknown ({message.fix_type})")
        status = Status.PASS if message.fix_type >= 3 else Status.WARN
        details = [f"Fix: {fix_name}", f"Satellites: {message.satellites_visible}"]
        if message.eph != 0xFFFF:
            details.append(f"EPH: {message.eph / 100:.2f} m")
        if message.epv != 0xFFFF:
            details.append(f"EPV: {message.epv / 100:.2f} m")
        if self.show_coordinates and message.lat != 0 and message.lon != 0:
            details.append(f"Coordinates: {message.lat / 1e7:.7f}, {message.lon / 1e7:.7f}")
        summary = "GPS telemetry received" if status is Status.PASS else "GPS telemetry received, but no 3D fix"
        return TestResult("GPS", status, summary, details)

    def test_shell_firmware(self, shell: PX4Shell) -> TestResult:
        output = shell.execute("ver all")
        return TestResult("PX4 SHELL FIRMWARE", Status.PASS, "ver all completed", self._lines(output, 6))

    def test_storage(self, shell: PX4Shell) -> TestResult:
        output = shell.execute("df") + "\n" + shell.execute("ls /fs/microsd")
        unavailable = any(marker in output.lower() for marker in ("no such file", "not found", "not mounted"))
        status = Status.WARN if unavailable else Status.PASS
        summary = "microSD directory listed" if not unavailable else "microSD filesystem is not available"
        return TestResult("STORAGE", status, summary, self._lines(output, 8))

    def test_logging(self, shell: PX4Shell) -> TestResult:
        output = shell.execute("logger status")
        unavailable = "not running" in output.lower() or "not started" in output.lower()
        inactive = "not logging" in output.lower()
        status = Status.WARN if unavailable or inactive else Status.PASS
        summary = "logger is recording" if not (unavailable or inactive) else "logger is available but not recording"
        return TestResult("LOGGING", status, summary, self._lines(output, 8))

    def test_onboard_sensors(self, shell: PX4Shell) -> list[TestResult]:
        topics = {
            "MAGNETOMETER": ("sensor_mag", True),
            "ACCELEROMETER": ("sensor_accel", False),
            "GYROSCOPE": ("sensor_gyro", False),
            "BAROMETER": ("sensor_baro", False),
            "IMU STATUS": ("sensors_status_imu", False),
        }
        results: list[TestResult] = []
        for section, (topic, optional) in topics.items():
            output = shell.execute(f"listener {topic} -n 1")
            observed = "timestamp:" in output
            status = Status.PASS if observed else (self._missing_peripheral_status() if optional else Status.FAIL)
            summary = f"{topic} sample received" if observed else f"No {topic} sample received"
            results.append(TestResult(section, status, summary, self._lines(output, 6)))
        return results

    def test_optional_links(self, shell: PX4Shell) -> list[TestResult]:
        results: list[TestResult] = []
        rc = shell.execute("listener input_rc -n 1")
        rc_present = "timestamp:" in rc and "rc_lost: True" not in rc and "timestamp_last_signal: 0" not in rc
        results.append(
            TestResult(
                "RC / ELRS",
                Status.PASS if rc_present else self._missing_peripheral_status(),
                "RC input is active" if rc_present else "No active RC signal (receiver may be disconnected)",
                self._lines(rc, 8),
            )
        )
        esc = shell.execute("listener esc_status -n 1")
        esc_present = "timestamp:" in esc and self._has_nonzero_numeric_field(esc, "esc_voltage")
        results.append(
            TestResult(
                "ESC TELEMETRY",
                Status.PASS if esc_present else self._missing_peripheral_status(),
                "ESC telemetry reports live electrical data" if esc_present else "No live ESC electrical telemetry (ESCs may be disconnected)",
                self._lines(esc, 8),
            )
        )
        dshot = shell.execute("dshot status")
        supported = "not found" not in dshot.lower() and "not running" not in dshot.lower()
        results.append(
            TestResult(
                "DSHOT OUTPUT",
                Status.PASS if supported else Status.SKIP,
                "dshot status received" if supported else "DShot driver is not active",
                self._lines(dshot, 6),
            )
        )
        return results

    def test_arm(self) -> TestResult:
        """Arm through the RadioMaster USB global-variable control path.

        This test is deliberately excluded from ``full`` and is reachable only
        through the CLI's separately safety-gated ``arm`` command.
        """
        if self.safety_context is None or not self.safety_context.vehicle_secured:
            return TestResult("ARM", Status.FAIL, "Vehicle safety confirmation was not accepted")
        if not self.controller_device:
            return TestResult("ARM", Status.FAIL, "No RadioMaster controller serial device was supplied")
        try:
            controller = _controller_control_module()
            # ARM is a switch gvar. The reviewed model mapping arms at +1024.
            controller.send_message(controller.Command.ARM, 1024, port=self.controller_device)
        except (OSError, ValueError) as error:
            return TestResult("ARM", Status.FAIL, f"Could not send RadioMaster arm command: {error}")

        deadline = time.monotonic() + self.connection.timeout
        armed = False
        while time.monotonic() < deadline:
            heartbeat = self.connection.receive("HEARTBEAT")
            if heartbeat is None:
                continue
            if heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                armed = True
                break

        try:
            # Always return the tested switch to its reviewed disarm value.
            controller.send_message(controller.Command.ARM, -1024, port=self.controller_device)
        except (OSError, ValueError) as error:
            return TestResult("ARM", Status.FAIL, f"Could not restore the ARM gvar to off: {error}")

        details = [
            f"Controller: {self.controller_device}",
            "Command: ARM = 1024",
            "ARM gvar restored to -1024 (disarm)",
        ]
        if armed:
            return TestResult("ARM", Status.PASS, "PX4 reported armed after the RadioMaster ARM gvar command", details)
        return TestResult("ARM", Status.FAIL, "PX4 did not report armed before the timeout", details)

    def run_full(self) -> list[TestResult]:
        results = [self.test_connection()]
        if results[0].status is Status.FAIL:
            return results
        for test in (
            self.test_vehicle_id,
            self.test_parameter_configuration,
            self.test_firmware,
            self.test_battery,
            self.test_gps,
        ):
            try:
                results.append(test())
            except (OSError, RuntimeError, ValueError) as error:
                results.append(TestResult(test.__name__.removeprefix("test_").upper(), Status.FAIL, str(error)))
        shell = PX4Shell(self.connection)
        try:
            results.append(self.test_shell_firmware(shell))
            results.append(self.test_storage(shell))
            results.append(self.test_logging(shell))
            results.extend(self.test_onboard_sensors(shell))
            results.extend(self.test_optional_links(shell))
        except ShellError as error:
            results.append(TestResult("PX4 SHELL", Status.FAIL, str(error)))
        finally:
            shell.close()
        return results

    @staticmethod
    def _lines(output: str, limit: int) -> list[str]:
        lines = [line.strip() for line in output.splitlines() if line.strip()]
        return lines[:limit]

    def _missing_peripheral_status(self) -> Status:
        return Status.FAIL if self.require_peripherals else Status.SKIP

    @staticmethod
    def _has_nonzero_numeric_field(output: str, field: str) -> bool:
        values = re.findall(rf"{re.escape(field)}:\s*(-?\d+(?:\.\d+)?)", output)
        return any(float(value) != 0.0 for value in values)

    @staticmethod
    def _battery_status_name(message: Any) -> str:
        state = mavutil.mavlink.enums["MAV_BATTERY_FUNCTION"].get(message.battery_function)
        function = state.name if state else str(message.battery_function)
        charge_state = getattr(message, "charge_state", None)
        if charge_state is None:
            return f"{function}; charge state not reported"
        charge_enum = mavutil.mavlink.enums["MAV_BATTERY_CHARGE_STATE"].get(charge_state)
        charge = charge_enum.name if charge_enum else str(charge_state)
        return f"{function}; {charge}"
