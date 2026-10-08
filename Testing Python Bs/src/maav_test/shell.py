"""A deliberately small, read-only PX4 NSH shell client over MAVLink."""

from __future__ import annotations

import time
import re

from pymavlink import mavutil

from .connection import PX4Connection


SHELL_DEVICE = mavutil.mavlink.SERIAL_CONTROL_DEV_SHELL
SHELL_PROMPT = "nsh> "
MAX_SERIAL_CONTROL_PAYLOAD = 70


class ShellError(RuntimeError):
    """Raised when the PX4 MAVLink shell cannot complete a command safely."""


class PX4Shell:
    """Execute a narrow allowlist of observational NSH commands.

    MAVLink SERIAL_CONTROL is powerful enough to execute arbitrary PX4 shell
    commands. This class intentionally rejects command separators and commands
    outside its allowlist; hazardous tests must get their own explicit workflow.
    """

    def __init__(self, connection: PX4Connection) -> None:
        self.connection = connection
        self._buffer = ""

    def execute(self, command: str) -> str:
        """Run one read-only command and return its terminal output.

        The initial newline establishes/synchronizes the shell prompt. A prompt
        terminates the result rather than relying on an arbitrary sleep.
        """
        normalized = self._validate(command)
        return self._execute(normalized)

    def execute_controlled(self, command: str) -> str:
        """Execute a command constructed by a safety-gated test workflow.

        This is intentionally not exposed by the generic ``shell`` CLI command.
        Only test code that has already applied its physical-safety interlocks
        may call it.
        """
        return self._execute(command)

    def _execute(self, command: str) -> str:
        self._send("\n")
        self._read_until_prompt()
        # PX4 may redraw the prompt more than once in one SERIAL_CONTROL frame.
        # That remainder belongs to shell startup, not to the command below.
        self._buffer = ""
        self._send(f"{command}\n")
        output = self._read_until_prompt()
        return self._remove_echo(output, command)

    def close(self) -> None:
        """Release shell control before the MAVLink connection is closed."""
        master = self.connection.master
        if master is not None:
            master.mav.serial_control_send(SHELL_DEVICE, 0, 0, 0, 0, [0] * MAX_SERIAL_CONTROL_PAYLOAD)

    def _send(self, text: str) -> None:
        master = self.connection._require_master()
        payload = text.encode("ascii")
        flags = (
            mavutil.mavlink.SERIAL_CONTROL_FLAG_EXCLUSIVE
            | mavutil.mavlink.SERIAL_CONTROL_FLAG_RESPOND
        )
        for offset in range(0, len(payload), MAX_SERIAL_CONTROL_PAYLOAD):
            chunk = payload[offset : offset + MAX_SERIAL_CONTROL_PAYLOAD]
            master.mav.serial_control_send(
                SHELL_DEVICE,
                flags,
                0,
                0,
                len(chunk),
                list(chunk) + [0] * (MAX_SERIAL_CONTROL_PAYLOAD - len(chunk)),
            )

    def _read_until_prompt(self) -> str:
        master = self.connection._require_master()
        deadline = time.monotonic() + self.connection.timeout
        while time.monotonic() < deadline:
            prompt_at = self._buffer.find(SHELL_PROMPT)
            if prompt_at >= 0:
                output = self._buffer[:prompt_at]
                self._buffer = self._buffer[prompt_at + len(SHELL_PROMPT) :]
                return output
            message = master.recv_match(type="SERIAL_CONTROL", blocking=True, timeout=0.25)
            if message is None or message.device != SHELL_DEVICE or message.count == 0:
                continue
            self._buffer += bytes(message.data[: message.count]).decode("utf-8", errors="replace")
        raise ShellError(f"PX4 shell did not return the {SHELL_PROMPT!r} prompt within {self.connection.timeout:.1f} seconds")

    @staticmethod
    def _remove_echo(output: str, command: str) -> str:
        """Remove a normal NSH echo while retaining command output verbatim."""
        cleaned = re.sub(r"\x1b\[[0-?]*[ -/]*[@-~]", "", output).replace("\r", "")
        if cleaned.startswith(command):
            cleaned = cleaned[len(command) :]
        return cleaned.lstrip("\n").rstrip()

    @staticmethod
    def _validate(command: str) -> str:
        normalized = " ".join(command.split())
        if not normalized:
            raise ShellError("A PX4 shell command is required")
        if any(character in normalized for character in ";|&><$`\\"):
            raise ShellError("Shell operators are not allowed")

        parts = normalized.split()
        allowed = (
            parts[0] == "ver"
            or parts[:2] == ["param", "show"]
            or parts[0] == "listener"
            or parts[:2] == ["logger", "status"]
            or parts[:2] == ["dshot", "status"]
            or parts[0] in {"ls", "df", "free", "top"}
            or parts[:2] == ["mavlink", "status"]
        )
        if not allowed:
            raise ShellError(
                "Command is outside the read-only shell allowlist. "
                "Allowed: ver, param show, listener, logger status, dshot status, ls, df, free, top, mavlink status."
            )
        return normalized
