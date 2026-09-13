#!/usr/bin/env python3
"""Project a FAST-LIO PCD cloud into a ROS PGM occupancy map."""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np


DTYPES = {
    ("F", 4): "<f4", ("F", 8): "<f8",
    ("I", 1): "<i1", ("I", 2): "<i2", ("I", 4): "<i4", ("I", 8): "<i8",
    ("U", 1): "<u1", ("U", 2): "<u2", ("U", 4): "<u4", ("U", 8): "<u8",
}


def read_header(handle):
    header = {}
    while True:
        raw = handle.readline()
        if not raw:
            raise ValueError("PCD header ended before DATA.")
        line = raw.decode("ascii").strip()
        if not line or line.startswith("#"):
            continue
        parts = line.split()
        header[parts[0].upper()] = parts[1:]
        if parts[0].upper() == "DATA":
            if len(parts) != 2:
                raise ValueError("Malformed DATA line.")
            return header, parts[1].lower()


def required(header, key):
    if key not in header:
        raise ValueError(f"PCD header is missing {key}.")
    return header[key]


def read_xyz(path):
    with path.open("rb") as handle:
        header, mode = read_header(handle)
        fields = required(header, "FIELDS")
        sizes = [int(value) for value in required(header, "SIZE")]
        types = required(header, "TYPE")
        counts = [int(value) for value in header.get("COUNT", ["1"] * len(fields))]
        point_count = int(required(header, "POINTS")[0])
        if not (len(fields) == len(sizes) == len(types) == len(counts)):
            raise ValueError("FIELDS, SIZE, TYPE, and COUNT lengths differ.")
        if not {"x", "y", "z"}.issubset(fields):
            raise ValueError("PCD must contain x, y, and z.")

        offsets, dtype_fields, offset = {}, [], 0
        for field, size, kind, count in zip(fields, sizes, types, counts):
            dtype = DTYPES.get((kind.upper(), size))
            if dtype is None:
                raise ValueError(f"Unsupported PCD field: {field} {kind}{size}.")
            offsets[field] = offset
            offset += count
            dtype_fields.append(
                (field, dtype) if count == 1 else (field, dtype, (count,))
            )

        if mode == "binary":
            cloud = np.fromfile(handle, dtype=np.dtype(dtype_fields), count=point_count)
            return tuple(np.asarray(cloud[key], dtype=np.float64) for key in ("x", "y", "z"))
        if mode == "ascii":
            values = np.loadtxt(
                handle,
                dtype=np.float64,
                usecols=(offsets["x"], offsets["y"], offsets["z"]),
            )
            values = np.atleast_2d(values)
            return values[:, 0], values[:, 1], values[:, 2]
        if mode == "binary_compressed":
            raise ValueError("binary_compressed PCD is unsupported; export binary or ASCII.")
        raise ValueError(f"Unsupported DATA mode: {mode}.")


def make_grid(x, y, z, resolution, z_min, z_max, padding, background, inflate_m):
    valid = (
        np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
        & (z >= z_min) & (z <= z_max)
    )
    x, y = x[valid], y[valid]
    if not x.size:
        raise ValueError("No points remain after filtering by height.")

    origin_x, origin_y = float(x.min() - padding), float(y.min() - padding)
    width = max(1, int(np.ceil((x.max() + padding - origin_x) / resolution)))
    height = max(1, int(np.ceil((y.max() + padding - origin_y) / resolution)))
    columns = np.clip(((x - origin_x) / resolution).astype(np.int64), 0, width - 1)
    rows = height - 1 - np.clip(
        ((y - origin_y) / resolution).astype(np.int64), 0, height - 1
    )
    occupied = np.unique(np.column_stack((rows, columns)), axis=0)
    grid = np.full((height, width), background, dtype=np.uint8)
    radius = int(np.ceil(inflate_m / resolution))
    for row_delta in range(-radius, radius + 1):
        for column_delta in range(-radius, radius + 1):
            if row_delta * row_delta + column_delta * column_delta > radius * radius:
                continue
            target_rows = occupied[:, 0] + row_delta
            target_columns = occupied[:, 1] + column_delta
            inside = (
                (target_rows >= 0) & (target_rows < height)
                & (target_columns >= 0) & (target_columns < width)
            )
            grid[target_rows[inside], target_columns[inside]] = 0
    return grid, origin_x, origin_y, int(x.size)


def write_map(prefix, grid, resolution, origin_x, origin_y):
    pgm_path, yaml_path = prefix.with_suffix(".pgm"), prefix.with_suffix(".yaml")
    pgm_path.parent.mkdir(parents=True, exist_ok=True)
    with pgm_path.open("wb") as handle:
        handle.write(
            f"P5\n# Generated from FAST-LIO PCD\n{grid.shape[1]} {grid.shape[0]}\n255\n".encode("ascii")
        )
        handle.write(grid.tobytes())
    yaml_path.write_text(
        "\n".join((
            f"image: {pgm_path.name}",
            f"resolution: {resolution:.6f}",
            f"origin: [{origin_x:.6f}, {origin_y:.6f}, 0.000000]",
            "negate: 0",
            "occupied_thresh: 0.65",
            "free_thresh: 0.196",
            "mode: trinary",
            "",
        )),
        encoding="ascii",
    )
    return pgm_path, yaml_path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input_pcd", type=Path)
    parser.add_argument("output_prefix", type=Path)
    parser.add_argument("--resolution", type=float, default=0.05)
    parser.add_argument("--z-min", type=float, default=-0.30)
    parser.add_argument("--z-max", type=float, default=0.30)
    parser.add_argument("--padding", type=float, default=0.50)
    parser.add_argument("--inflate-m", type=float, default=0.05)
    parser.add_argument("--background", choices=("unknown", "free"), default="unknown")
    args = parser.parse_args()
    if args.resolution <= 0 or args.padding < 0 or args.inflate_m < 0:
        parser.error("resolution must be positive; padding and inflate-m cannot be negative.")
    if args.z_min >= args.z_max:
        parser.error("z-min must be smaller than z-max.")
    if not args.input_pcd.is_file():
        parser.error(f"PCD does not exist: {args.input_pcd}")

    x, y, z = read_xyz(args.input_pcd)
    background = 205 if args.background == "unknown" else 254
    grid, origin_x, origin_y, kept = make_grid(
        x, y, z, args.resolution, args.z_min, args.z_max,
        args.padding, background, args.inflate_m,
    )
    pgm_path, yaml_path = write_map(
        args.output_prefix, grid, args.resolution, origin_x, origin_y
    )
    print(f"Converted {kept} points to {grid.shape[1]}x{grid.shape[0]} cells.")
    print(f"PGM: {pgm_path}")
    print(f"YAML: {yaml_path}")


if __name__ == "__main__":
    main()
