"""Serial discovery and the thin, safe MAVLink transport wrapper."""

from __future__ import annotations

import time
import struct
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from pymavlink import mavutil
from serial.tools import list_ports


USB_DEVICE_HINTS = ("usbmodem", "usbserial", "cu.usb", "ttyacm", "ttyusb")


class DeviceDiscoveryError(RuntimeError):
    """Raised when discovery cannot safely choose one serial device."""


@dataclass(frozen=True)
class SerialPort:
    device: str
    description: str
    hardware_id: str


def available_serial_ports() -> list[SerialPort]:
    """Return serial ports without applying a macOS- or Linux-only policy."""
    return [
        SerialPort(port.device, port.description or "Unknown", port.hwid or "Unknown")
        for port in list_ports.comports()
    ]


def discover_pixhawk_device() -> str:
    """Select one plausible USB MAVLink device, or explain why we cannot."""
    ports = available_serial_ports()
    candidates = [
        port
        for port in ports
        if any(hint in port.device.lower() for hint in USB_DEVICE_HINTS)
        or "pixhawk" in port.description.lower()
    ]
    if len(candidates) == 1:
        return candidates[0].device
    if not candidates:
        raise DeviceDiscoveryError(
            "No likely USB serial device found. Use --list-ports, then pass --device PATH."
        )
    choices = ", ".join(port.device for port in candidates)
    raise DeviceDiscoveryError(
        f"Multiple likely USB serial devices found ({choices}). Pass --device PATH explicitly."
    )


def _decode_text(value: Any) -> str:
    """Decode fixed-width MAVLink byte fields and remove NUL padding."""
    if isinstance(value, bytes):
        return value.split(b"\0", 1)[0].decode("utf-8", errors="replace")
    return str(value).split("\0", 1)[0]


def decode_parameter_value(message: Any) -> int | float:
    """Decode a MAVLink parameter using its declared wire type.

    PX4 bytewise-encodes integer parameters in PARAM_VALUE's float32 field.
    Reading ``param_value`` as a normal Python float would therefore render an
    integer value of 1 as approximately 1.4e-45.
    """
    integer_formats = {
        mavutil.mavlink.MAV_PARAM_TYPE_UINT8: "<B",
        mavutil.mavlink.MAV_PARAM_TYPE_INT8: "<b",
        mavutil.mavlink.MAV_PARAM_TYPE_UINT16: "<H",
        mavutil.mavlink.MAV_PARAM_TYPE_INT16: "<h",
        mavutil.mavlink.MAV_PARAM_TYPE_UINT32: "<I",
        mavutil.mavlink.MAV_PARAM_TYPE_INT32: "<i",
    }
    value_format = integer_formats.get(message.param_type)
    if value_format is None:
        return message.param_value
    packed = struct.pack("<f", message.param_value)
    return struct.unpack(value_format, packed[: struct.calcsize(value_format)])[0]


class PX4Connection:
    """Owns a MAVLink serial connection and directed information requests."""

    def __init__(self, device: str, baud: int, timeout: float) -> None:
        self.device = device
        self.baud = baud
        self.timeout = timeout
        self.master: Any | None = None
        self.target_system: int | None = None
        self.target_component: int | None = None

    def open(self) -> None:
        self.master = mavutil.mavlink_connection(
            self.device,
            baud=self.baud,
            source_system=255,
            source_component=190,
            autoreconnect=True,
        )

    def close(self) -> None:
        if self.master is not None:
            self.master.close()
            self.master = None

    def __enter__(self) -> "PX4Connection":
        self.open()
        return self

    def __exit__(self, *_: object) -> None:
        self.close()

    def wait_for_heartbeat(self) -> Any:
        """Wait for a vehicle heartbeat and record the IDs for later requests."""
        master = self._require_master()
        heartbeat = master.wait_heartbeat(timeout=self.timeout)
        if heartbeat is None:
            raise TimeoutError(f"No MAVLink heartbeat received within {self.timeout:.1f} seconds")
        self.target_system = master.target_system
        # pymavlink updates target_system here but can retain its default
        # broadcast component (0). Address the component that sent the
        # heartbeat so parameter and command requests are properly directed.
        self.target_component = heartbeat.get_srcComponent()
        return heartbeat

    def request_parameter(self, name: str) -> Any | None:
        master = self._require_target()
        master.mav.param_request_read_send(
            self.target_system, self.target_component, name.encode("ascii"), -1
        )
        deadline = time.monotonic() + self.timeout
        while time.monotonic() < deadline:
            message = master.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.25)
            if message is not None and _decode_text(message.param_id) == name:
                return message
        return None

    def request_all_parameters(self, timeout: float = 30.0) -> dict[str, int | float]:
        """Fetch the complete parameter set through the MAVLink parameter protocol."""
        master = self._require_target()
        master.mav.param_request_list_send(self.target_system, self.target_component)
        deadline = time.monotonic() + timeout
        parameters: dict[str, int | float] = {}
        expected_count: int | None = None
        while time.monotonic() < deadline:
            message = master.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
            if message is None:
                if expected_count is not None and len(parameters) >= expected_count:
                    break
                continue
            expected_count = message.param_count
            parameters[_decode_text(message.param_id)] = decode_parameter_value(message)
            if len(parameters) >= expected_count:
                break
        if not parameters:
            raise TimeoutError("No PX4 parameters received")
        if expected_count is not None and len(parameters) < expected_count:
            raise TimeoutError(f"Received {len(parameters)} of {expected_count} PX4 parameters")
        return parameters

    def request_autopilot_version(self) -> Any | None:
        master = self._require_target()
        master.mav.command_long_send(
            self.target_system,
            self.target_component,
            mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
            0,
            float(mavutil.mavlink.MAVLINK_MSG_ID_AUTOPILOT_VERSION),
            0, 0, 0, 0, 0, 0,
        )
        return self.receive("AUTOPILOT_VERSION")

    def request_telemetry(self, message_id: int, message_name: str, interval_us: int = 1_000_000) -> Any | None:
        """Ask for a safe telemetry stream, then wait for one message from it."""
        master = self._require_target()
        master.mav.command_long_send(
            self.target_system,
            self.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            float(message_id), float(interval_us), 0, 0, 0, 0, 0,
        )
        return self.receive(message_name)

    def receive(self, message_name: str) -> Any | None:
        master = self._require_master()
        return master.recv_match(type=message_name, blocking=True, timeout=self.timeout)

    def _require_master(self) -> Any:
        if self.master is None:
            raise RuntimeError("Connection has not been opened")
        return self.master

    def _require_target(self) -> Any:
        if self.target_system is None or self.target_component is None:
            raise RuntimeError("A heartbeat is required before sending directed requests")
        return self._require_master()


def git_hash_from_version(version: Any) -> str:
    """Render the optional 8-byte MAVLink version hash without assuming UTF-8."""
    raw = bytes(getattr(version, "flight_custom_version", b""))
    return raw.hex() if raw and any(raw) else "not reported"


def device_name(device: str) -> str:
    """Keep future display code independent of platform-specific device paths."""
    return Path(device).name
