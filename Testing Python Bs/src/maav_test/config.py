"""Golden PX4 parameter snapshots and comparisons."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any


def write_snapshot(path: Path, parameters: dict[str, int | float]) -> None:
    path.write_text(
        json.dumps({"format": 1, "parameters": dict(sorted(parameters.items()))}, indent=2) + "\n",
        encoding="utf-8",
    )


def read_snapshot(path: Path) -> dict[str, int | float]:
    try:
        document: Any = json.loads(path.read_text(encoding="utf-8"))
        parameters = document["parameters"]
    except (OSError, KeyError, TypeError, json.JSONDecodeError) as error:
        raise ValueError(f"Could not read golden configuration {path}: {error}") from error
    if not isinstance(parameters, dict) or not all(
        isinstance(name, str) and isinstance(value, (int, float))
        for name, value in parameters.items()
    ):
        raise ValueError(f"Golden configuration {path} must contain numeric parameters")
    return parameters


def differences(
    golden: dict[str, int | float], current: dict[str, int | float]
) -> tuple[list[str], list[str]]:
    """Return missing names and value differences; extra current keys are allowed."""
    missing = sorted(set(golden) - set(current))
    changed = [
        f"{name}: expected {golden[name]!r}, got {current[name]!r}"
        for name in sorted(set(golden) & set(current))
        if golden[name] != current[name]
    ]
    return missing, changed
