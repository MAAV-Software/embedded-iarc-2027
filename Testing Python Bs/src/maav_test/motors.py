"""Explicitly safety-gated motor-output checks.

These tests are never included in ``full``. They are intended only for a
restrained, props-removed aircraft with a verified power source.
"""

from __future__ import annotations

import re
import time

from .models import SafetyContext, Status, TestResult
from .shell import PX4Shell


MAX_POWER = 0.15
MAX_DURATION_SECONDS = 5.0


class MotorSafetyError(ValueError):
    """Raised before any actuator output when required interlocks are absent."""


def validate_motor_request(
    safety_confirmed: bool,
    motors: list[int],
    all_motors: bool,
    power: float,
    duration: float,
) -> None:
    if not safety_confirmed:
        raise MotorSafetyError("Motor safety confirmation was not accepted")
    if not motors and not all_motors:
        raise MotorSafetyError("Select one or more --motor values, or pass --all-motors")
    if all_motors and motors:
        raise MotorSafetyError("Use either --motor or --all-motors, not both")
    if any(motor < 1 or motor > 12 for motor in motors):
        raise MotorSafetyError("Motor numbers must be in the range 1..12")
    if not 0 < power <= MAX_POWER:
        raise MotorSafetyError(f"--motor-power must be greater than 0 and no more than {MAX_POWER}")
    if not 0 < duration <= MAX_DURATION_SECONDS:
        raise MotorSafetyError(f"--motor-duration must be greater than 0 and no more than {MAX_DURATION_SECONDS} seconds")


def confirm_vehicle_safety() -> bool:
    """Obtain the operator's physical-safety acknowledgement before MAVLink use."""
    try:
        answer = input("Are the props removed or is the drone securely strapped down? [y/N] ")
    except EOFError:
        return False
    return answer.strip().lower() in {"y", "yes"}


class MotorTests:
    def __init__(self, shell: PX4Shell, safety_context: SafetyContext) -> None:
        self.shell = shell
        self.safety_context = safety_context

    def run(self, motors: list[int], all_motors: bool, power: float, duration: float) -> list[TestResult]:
        if not self.safety_context.vehicle_secured:
            raise MotorSafetyError("Motor output is unavailable because suite-start safety confirmation was declined")
        esc_before = self.shell.execute("listener esc_status -n 1")
        selected = motors or self._configured_motors(esc_before)
        if not selected:
            return [TestResult("MOTOR TEST", Status.FAIL, "No configured motor outputs found")]

        results = [
            TestResult(
                "MOTOR TEST SAFETY",
                Status.PASS,
                f"Props-removed interlock accepted; testing motors {selected} at {power:.2f} for {duration:.1f}s",
            )
        ]
        for motor in selected:
            imu_before = self.shell.execute("listener vehicle_imu_status -n 1")
            output = self.shell.execute_controlled(
                f"actuator_test set -m {motor} -v {power:.3f} -t {duration:.1f}"
            )
            # PX4 accepts the command immediately and applies its own timeout
            # asynchronously. Keep this test's motors strictly sequential.
            time.sleep(duration)
            esc_after = self.shell.execute("listener esc_status -n 1")
            imu_after = self.shell.execute("listener vehicle_imu_status -n 1")
            results.append(self._motor_result(motor, output, esc_after, imu_before, imu_after))
        return results

    @staticmethod
    def _configured_motors(esc_output: str) -> list[int]:
        count = re.search(r"esc_count:\s*(\d+)", esc_output)
        return list(range(1, int(count.group(1)) + 1)) if count else []

    @staticmethod
    def _motor_result(
        motor: int, command_output: str, esc_output: str, imu_before: str, imu_after: str
    ) -> TestResult:
        command_failed = any(token in command_output.lower() for token in ("error", "failed", "not found"))
        rpm = MotorTests._field(esc_output, "esc_rpm", motor - 1)
        voltage = MotorTests._field(esc_output, "esc_voltage", motor - 1)
        current = MotorTests._field(esc_output, "esc_current", motor - 1)
        temperature = MotorTests._field(esc_output, "esc_temperature", motor - 1)
        details = [f"PX4 actuator command: {command_output.strip() or 'accepted without text output'}"]
        if rpm is not None:
            details.append(f"ESC RPM: {rpm}")
        if voltage is not None:
            details.append(f"ESC voltage: {voltage} V")
        if current is not None:
            details.append(f"ESC current: {current} A")
        if temperature is not None:
            details.append(f"ESC temperature: {temperature} C")
        details.append("IMU snapshots captured before and after spin; compare vibration in future thresholded analysis")
        if command_failed:
            return TestResult(f"MOTOR {motor}", Status.FAIL, "PX4 rejected motor test command", details)
        has_electrical_telemetry = any(
            value is not None and float(value) != 0.0 for value in (voltage, current)
        )
        if rpm is None or float(rpm) == 0:
            return TestResult(f"MOTOR {motor}", Status.WARN, "Motor command completed; no non-zero RPM telemetry", details)
        if not has_electrical_telemetry:
            return TestResult(
                f"MOTOR {motor}",
                Status.WARN,
                "RPM was reported without live electrical telemetry; cannot verify motor movement",
                details,
            )
        return TestResult(f"MOTOR {motor}", Status.PASS, "Motor command completed with RPM telemetry", details)

    @staticmethod
    def _field(output: str, field: str, index: int) -> str | None:
        reports = re.finditer(r"esc\[(\d+)\].*?(?=\n\s*esc\[|\Z)", output, flags=re.DOTALL)
        # listener output can be truncated by a firmware-specific layout; fall
        # back to the first occurrence only when a per-ESC block is unavailable.
        for report in reports:
            if int(report.group(1)) == index:
                match = re.search(rf"{field}:\s*(-?\d+(?:\.\d+)?)", report.group(0))
                return match.group(1) if match else None
        match = re.search(rf"{field}:\s*(-?\d+(?:\.\d+)?)", output)
        return match.group(1) if match else None
