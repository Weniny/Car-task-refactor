"""Content fingerprints that bind a frozen trajectory to its planning inputs."""

from __future__ import annotations

import hashlib
from pathlib import Path
from typing import Any, Mapping

import yaml


SourcePaths = Mapping[str, str | Path]


def build_source_manifest(source_paths: SourcePaths) -> dict[str, dict[str, object]]:
    manifest: dict[str, dict[str, object]] = {}
    for name, raw_path in sorted(source_paths.items()):
        path = Path(raw_path)
        payload = path.read_bytes()
        manifest[name] = {
            "sha256": hashlib.sha256(payload).hexdigest(),
            "size_bytes": len(payload),
        }
    return manifest


def validate_source_manifest(
    artifact: Mapping[str, Any],
    source_paths: SourcePaths,
) -> None:
    expected = artifact.get("source_manifest")
    if not isinstance(expected, dict):
        raise ValueError("trajectory artifact has no source_manifest")
    actual = build_source_manifest(source_paths)
    problems: list[str] = []
    for name, actual_entry in actual.items():
        expected_entry = expected.get(name)
        if not isinstance(expected_entry, dict):
            problems.append(f"{name}:missing")
            continue
        if expected_entry.get("sha256") != actual_entry["sha256"]:
            problems.append(f"{name}:sha256_mismatch")
    if problems:
        raise ValueError("trajectory source mismatch: " + ", ".join(problems))


def resolve_trajectory_source_paths(
    *,
    route_file: str | Path,
    semantic_map_file: str | Path,
    planning_params_file: str | Path,
    optimizer_params_file: str | Path,
) -> dict[str, Path]:
    """Resolve route/config plus the occupancy YAML and referenced image."""

    route_path = Path(route_file).resolve()
    semantic_path = Path(semantic_map_file).resolve()
    planning_path = Path(planning_params_file).resolve()
    optimizer_path = Path(optimizer_params_file).resolve()
    planning = _load_yaml(planning_path)
    semantic = _load_yaml(semantic_path)
    map_reference = planning.get("global_planner", {}).get("map_file") or semantic.get(
        "source_map"
    )
    if not map_reference:
        raise ValueError("planning inputs do not identify an occupancy map")
    occupancy_map = _resolve_reference(str(map_reference), planning_path)
    occupancy_config = _load_yaml(occupancy_map)
    image_reference = occupancy_config.get("image")
    if not image_reference:
        raise ValueError(f"occupancy map {occupancy_map} has no image")
    occupancy_image = _resolve_reference(str(image_reference), occupancy_map)
    return {
        "route": route_path,
        "semantic_map": semantic_path,
        "planning_params": planning_path,
        "optimizer_params": optimizer_path,
        "occupancy_map": occupancy_map,
        "occupancy_image": occupancy_image,
    }


def _resolve_reference(reference: str, anchor_file: Path) -> Path:
    candidate = Path(reference)
    if candidate.is_absolute():
        resolved = candidate.resolve()
        if resolved.is_file():
            return resolved
        raise ValueError(f"source file does not exist: {resolved}")
    candidates = [anchor_file.parent / candidate, Path.cwd() / candidate]
    candidates.extend(parent / candidate for parent in anchor_file.parents)
    for path in candidates:
        resolved = path.resolve()
        if resolved.is_file():
            return resolved
    raise ValueError(f"cannot resolve source file {reference} from {anchor_file}")


def _load_yaml(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict):
        raise ValueError(f"{path} did not contain a YAML mapping")
    return data
