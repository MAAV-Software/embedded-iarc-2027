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

ESC_TELEMETRY_SAMPLE_COUNT = 55
ESC_TELEMETRY_TIMEOUT_SECONDS = 45.0


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
        require_peripherals: bool = True,
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
        live_voltages = [voltage for voltage in voltages if voltage > 0]
        details = [f"Battery ID: {message.id}", f"Status: {self._battery_status_name(message)}"]
        if voltages:
            details.append(f"Voltage: {sum(voltages) / 1000:.2f} V")
        else:
            details.append("Voltage: not reported")
        if message.current_battery != -1:
            details.append(f"Current: {message.current_battery / 100:.2f} A")
        if message.battery_remaining != -1:
            details.append(f"Remaining: {message.battery_remaining}%")
        status = Status.PASS if live_voltages else self._missing_peripheral_status()
        summary = (
            "Live battery voltage received"
            if live_voltages
            else "Battery telemetry received, but the pack voltage is zero or unavailable"
        )
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
        results.append(self.test_bidirectional_dshot(shell))
        return results

    def test_bidirectional_dshot(self, shell: PX4Shell) -> TestResult:
        """Verify bidirectional DShot replies separately from pack power.

        The flight controller sends DShot on USB power alone, but ESCs only
        reply when they are powered, so this test needs the pack connected.
        The BATTERY test verifies pack voltage through the flight controller's
        power monitor; this test verifies the bidirectional-DShot data path
        through per-channel reply counters.
        """
        name = "BIDIRECTIONAL DSHOT / ESC TELEMETRY"

        # Learn how many ESCs PX4 expects before taking the "before" snapshot,
        # so that snapshot can be retried until every channel parses.
        probe = self._esc_status_samples(shell.execute("listener esc_status -n 1"))
        if not probe:
            return TestResult(name, self._missing_peripheral_status(), "No ESC status samples received")
        esc_count = probe[0]["esc_count"]
        channels = list(range(esc_count))

        dshot_before, reply_before = self._read_dshot_reply_counts(shell, channels)
        if "bidirectional dshot" not in dshot_before.lower() and not reply_before:
            return TestResult(name, Status.FAIL, "Bidirectional DShot is not enabled", self._lines(dshot_before, 12))

        original_timeout = self.connection.timeout
        self.connection.timeout = max(original_timeout, ESC_TELEMETRY_TIMEOUT_SECONDS)
        try:
            telemetry = shell.execute(f"listener esc_status -n {ESC_TELEMETRY_SAMPLE_COUNT}")
        finally:
            self.connection.timeout = original_timeout
        dshot_after, reply_after = self._read_dshot_reply_counts(shell, channels)

        unparsed = [
            channel for channel in channels if channel not in reply_before or channel not in reply_after
        ]
        if unparsed:
            # Never treat an unreadable counter as zero: that produces bogus deltas.
            return TestResult(
                name,
                Status.FAIL,
                "Could not read DShot reply counters for "
                + ", ".join(f"ch{channel}" for channel in unparsed),
                ["dshot status (before):", *self._lines(dshot_before, 20),
                 "dshot status (after):", *self._lines(dshot_after, 20)],
            )

        samples = self._esc_status_samples(telemetry)
        if not samples:
            return TestResult(
                name,
                self._missing_peripheral_status(),
                "No ESC status samples received",
                self._lines(telemetry, 12),
            )

        expected_mask = (1 << esc_count) - 1
        reply_deltas = {channel: reply_after[channel] - reply_before[channel] for channel in channels}
        fresh_counters = len(samples) > 1 and all(
            later["counter"] > earlier["counter"] for earlier, later in zip(samples, samples[1:])
        )
        online = all((sample["online_flags"] & expected_mask) == expected_mask for sample in samples)
        fresh_reports = all(
            len({sample["reports"].get(index, {}).get("timestamp", 0) for sample in samples}) > 1
            for index in range(esc_count)
        )
        replies_advanced = all(delta > 0 for delta in reply_deltas.values())
        details = [
            f"ESC samples: {len(samples)} requested {ESC_TELEMETRY_SAMPLE_COUNT}",
            f"ESC count: {esc_count}; online flags: 0x{samples[-1]['online_flags']:x}",
            f"ESC status counters advanced: {'yes' if fresh_counters else 'no'}",
            f"ESC report timestamps advanced: {'yes' if fresh_reports else 'no'}",
            *(
                f"Output {index + 1} ({label}): read {reply_after[index]} (+{reply_deltas[index]} during test)"
                + ("  <-- NO REPLIES" if reply_deltas[index] <= 0 else "")
                for index, (label, _) in enumerate(self._dshot_reply_lines(dshot_after)[:esc_count])
            ),
            "Pack voltage is validated separately by the BATTERY test (not esc_status).",
        ]
        if fresh_counters and online and fresh_reports and replies_advanced:
            return TestResult(
                "BIDIRECTIONAL DSHOT / ESC TELEMETRY",
                Status.PASS,
                "Bidirectional DShot replies verified on every configured output",
                details,
            )

        missing: list[str] = []
        if not replies_advanced:
            missing.append("no new DShot replies from one or more ESC channels")
        if not online:
            missing.append("one or more ESCs were not online")
        if not fresh_counters or not fresh_reports:
            missing.append("ESC telemetry was not fresh across the sample window")
        return TestResult(
            "BIDIRECTIONAL DSHOT / ESC TELEMETRY",
            self._missing_peripheral_status(),
            "; ".join(missing),
            details,
        )

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

        armed, arm_messages = self._wait_for_arm_result()

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
        details.extend(f"PX4: {message}" for message in arm_messages)
        if armed:
            return TestResult("ARM", Status.PASS, "PX4 reported armed after the RadioMaster ARM gvar command", details)
        details.extend(self._preflight_failure_details(arm_messages))
        return TestResult("ARM", Status.FAIL, "PX4 did not report armed before the timeout", details)

    def _wait_for_arm_result(self) -> tuple[bool, list[str]]:
        """Wait for PX4's arm flag while retaining its diagnostic messages."""
        master = self.connection._require_master()
        deadline = time.monotonic() + self.connection.timeout
        messages: list[str] = []
        while time.monotonic() < deadline:
            message = master.recv_match(blocking=True, timeout=min(0.25, deadline - time.monotonic()))
            if message is None:
                continue
            if message.get_type() == "STATUSTEXT":
                text = getattr(message, "text", "")
                if isinstance(text, bytes):
                    text = text.decode("utf-8", errors="replace")
                text = str(text).strip("\x00 ")
                if text and text not in messages and len(messages) < 20:
                    messages.append(text)
                continue
            if message.get_type() == "HEARTBEAT" and (
                message.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
            ):
                return True, messages
        return False, messages

    def _preflight_failure_details(self, arm_messages: list[str]) -> list[str]:
        """Collect read-only PX4 state that explains an arm rejection.

        These commands run only after restoring the RadioMaster ARM gvar. They
        neither alter PX4 parameters nor command actuators.
        """
        shell = PX4Shell(self.connection)
        commands = (
            ("PX4 preflight check", "commander check", 20),
            ("PX4 vehicle status", "listener vehicle_status -n 1", 40),
            ("PX4 estimator status", "listener estimator_status -n 1", 80),
            ("PX4 estimator flags", "listener estimator_status_flags -n 1", 80),
            ("PX4 magnetometer sample", "listener sensor_mag -n 1", 20),
            ("PX4 RC input", "listener input_rc -n 1", 30),
        )
        details: list[str] = []
        try:
            for label, command, limit in commands:
                try:
                    output = shell.execute_controlled(command)
                except (OSError, ShellError, TimeoutError) as error:
                    details.append(f"{label} unavailable: {error}")
                    continue
                lines = self._lines(output, limit)
                if lines:
                    details.extend([f"{label}:", *lines])
                else:
                    details.append(f"{label}: no output")

            try:
                log_lines = self._preflight_log_lines(shell.execute_controlled("dmesg"))
            except (OSError, ShellError, TimeoutError) as error:
                details.append(f"PX4 relevant log entries unavailable: {error}")
            else:
                details.extend(["PX4 relevant log entries:", *log_lines] if log_lines else ["PX4 relevant log entries: none found"])
        finally:
            shell.close()
        if arm_messages:
            details.insert(0, "PX4 arm-rejection messages were captured above.")
        return details

    @staticmethod
    def _preflight_log_lines(output: str) -> list[str]:
        """Keep the useful, compact subset of the PX4 boot/log buffer."""
        markers = (
            "preflight",
            "arming denied",
            "health_and_arming_checks",
            "magnet",
            "compass",
            "qmc",
            "ekf",
            "calibrat",
            "no device",
        )
        lines = [line.strip() for line in output.splitlines() if any(marker in line.lower() for marker in markers)]
        return lines[-30:]

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
    def _dshot_reply_counts(output: str) -> dict[int, int]:
        """Return bidirectional-DShot reply counters by output channel.

        Tolerates format variants such as ``Channel 0: read 123`` and
        ``Channel 0: online, read 123, failed nibble 0, ...``.
        """
        return {index: count for index, (_, count) in enumerate(PX4Tests._dshot_reply_lines(output))}

    @staticmethod
    def _dshot_reply_lines(output: str) -> list[tuple[str, int]]:
        """Return ``(label, read_count)`` per DShot output, in printed order.

        PX4 may number channels per timer (``Timer 0 channel 1: read 0``), so
        channel numbers repeat across timers. Outputs are therefore indexed by
        the order they are printed, which follows the motor output order.
        """
        lines: list[tuple[str, int]] = []
        for match in re.finditer(
            r"(?:timer\s+(\d+)\W+)?channel\s+(\d+)\b[^\n]*?\bread\s+(\d+)",
            output,
            flags=re.IGNORECASE,
        ):
            timer, channel, count = match.groups()
            label = f"timer {timer} ch {channel}" if timer is not None else f"ch {channel}"
            lines.append((label, int(count)))
        return lines

    def _read_dshot_reply_counts(
        self, shell: PX4Shell, channels: list[int], attempts: int = 3
    ) -> tuple[str, dict[int, int]]:
        """Run ``dshot status`` until every expected channel's counter parses.

        Long MAVLink-shell output can arrive incomplete, silently dropping
        ``Channel`` lines, so a single read is not trustworthy.
        """
        output, counts = "", {}
        for _ in range(attempts):
            output = shell.execute("dshot status")
            counts = self._dshot_reply_counts(output)
            if all(channel in counts for channel in channels):
                break
        return output, counts

    @staticmethod
    def _esc_status_samples(output: str) -> list[dict[str, Any]]:
        """Parse PX4 ``listener esc_status`` output into small sample records."""
        samples: list[dict[str, Any]] = []
        for block in re.split(r"(?=TOPIC:\s+esc_status\b)", output):
            counter = re.search(r"\bcounter:\s*(\d+)", block)
            esc_count = re.search(r"\besc_count:\s*(\d+)", block)
            online_flags = re.search(r"\besc_online_flags:\s*(\d+)", block)
            if counter is None or esc_count is None or online_flags is None:
                continue
            reports: dict[int, dict[str, int | float]] = {}
            for report in re.finditer(
                r"esc\[(\d+)\]\s+\(esc_report\):(?P<body>.*?)(?=\n\s*esc\[|\Z)",
                block,
                flags=re.DOTALL,
            ):
                timestamp = re.search(r"\btimestamp:\s*(\d+)", report.group("body"))
                voltage = re.search(r"\besc_voltage:\s*(-?\d+(?:\.\d+)?)", report.group("body"))
                if timestamp is None or voltage is None:
                    continue
                reports[int(report.group(1))] = {
                    "timestamp": int(timestamp.group(1)),
                    "voltage": float(voltage.group(1)),
                }
            samples.append(
                {
                    "counter": int(counter.group(1)),
                    "esc_count": int(esc_count.group(1)),
                    "online_flags": int(online_flags.group(1)),
                    "reports": reports,
                }
            )
        return samples

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
