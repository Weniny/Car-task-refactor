#!/usr/bin/env python3
"""Verify frozen release files without importing ROS or starting any process."""

import hashlib
import json
from pathlib import Path
import runpy


def main():
    root = Path(__file__).resolve().parents[1]
    release = json.loads((root / "provenance/RELEASE_MANIFEST.json").read_text(encoding="utf-8"))
    for name, expected in release["additional_files"].items():
        path = (root / name).resolve()
        if not path.is_relative_to(root) or not path.is_file():
            raise SystemExit("Release file missing or outside workspace: " + name)
        if hashlib.sha256(path.read_bytes()).hexdigest() != expected:
            raise SystemExit("Release hash mismatch: " + name)
    runpy.run_path(str(root / "runtime/scripts/verify_candidate.py"), run_name="__main__")
    print("Stage 1 release integrity PASS:", release["id"])
    print("No ROS nodes, navigation or vehicle motion were started.")


if __name__ == "__main__":
    main()
