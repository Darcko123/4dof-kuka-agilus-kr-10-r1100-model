#!/usr/bin/env python3

from __future__ import annotations

import sys
from pathlib import Path


def _read(path: Path) -> str | None:
    try:
        text = path.read_text(encoding="utf-8").strip()
    except OSError:
        return None
    return text or None


def _resolve_version() -> str:

    meipass = getattr(sys, "_MEIPASS", None)
    if meipass:
        version = _read(Path(meipass) / "VERSION")
        if version:
            return version
    version = _read(Path(__file__).resolve().parents[2] / "VERSION")
    if version:
        return version
    try:
        from importlib.metadata import PackageNotFoundError, version as _dist_version

        return _dist_version("kukaagilus")
    except (ImportError, PackageNotFoundError):
        pass

    return "unknown"


__version__ = _resolve_version()
