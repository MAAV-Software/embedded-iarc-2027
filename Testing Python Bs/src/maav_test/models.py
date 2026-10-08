"""Small, presentation-independent result types for acceptance checks."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum


class Status(str, Enum):
    PASS = "PASS"
    FAIL = "FAIL"
    WARN = "WARN"
    SKIP = "SKIP"


@dataclass(frozen=True)
class SafetyContext:
    """Operator-provided physical-safety state for one test-suite invocation."""

    vehicle_secured: bool


@dataclass(frozen=True)
class TestResult:
    section: str
    status: Status
    summary: str
    details: list[str] = field(default_factory=list)
