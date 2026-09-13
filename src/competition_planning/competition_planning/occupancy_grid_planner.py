"""Occupancy-grid A* backend for semantic global route segments."""

from __future__ import annotations

from dataclasses import dataclass
import heapq
import math
from pathlib import Path
from typing import Any, Sequence

import yaml

from competition_planning.semantic_planner import PathPoint


class GridPlanningError(RuntimeError):
    """Raised when the occupancy-grid backend cannot produce a safe path."""


@dataclass(frozen=True)
class OccupancyGridMap:
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    occupied: tuple[bool, ...]

    @classmethod
    def from_yaml(cls, map_file: str | Path) -> "OccupancyGridMap":
        map_path = Path(map_file)
        with map_path.open("r", encoding="utf-8") as stream:
            metadata = yaml.safe_load(stream)
        if not isinstance(metadata, dict):
            raise GridPlanningError(f"{map_path} did not contain a YAML mapping")

        image_file = Path(str(metadata.get("image", "")))
        if not image_file.is_absolute():
            image_file = map_path.parent / image_file
        width, height, pixels = _read_pgm(image_file)
        resolution = float(metadata["resolution"])
        origin = metadata.get("origin", [0.0, 0.0, 0.0])
        occupied_thresh = float(metadata.get("occupied_thresh", 0.65))
        negate = int(metadata.get("negate", 0))

        occupied = []
        for value in pixels:
            occupancy_probability = value / 255.0 if negate else (255 - value) / 255.0
            occupied.append(occupancy_probability > occupied_thresh)

        return cls(
            width=width,
            height=height,
            resolution=resolution,
            origin_x=float(origin[0]),
            origin_y=float(origin[1]),
            occupied=tuple(occupied),
        )

    def index(self, cell: tuple[int, int]) -> int:
        col, row = cell
        return row * self.width + col

    def contains(self, cell: tuple[int, int]) -> bool:
        col, row = cell
        return 0 <= col < self.width and 0 <= row < self.height

    def world_to_cell(self, x: float, y: float) -> tuple[int, int]:
        col = int(math.floor((x - self.origin_x) / self.resolution))
        row_from_bottom = int(math.floor((y - self.origin_y) / self.resolution))
        row = self.height - 1 - row_from_bottom
        return col, row

    def cell_to_world(self, cell: tuple[int, int]) -> tuple[float, float]:
        col, row = cell
        x = self.origin_x + (col + 0.5) * self.resolution
        y = self.origin_y + (self.height - row - 0.5) * self.resolution
        return x, y

    def inflated_blocked(self, inflation_radius_m: float) -> tuple[bool, ...]:
        radius_cells = int(math.ceil(max(0.0, inflation_radius_m) / self.resolution))
        blocked = bytearray(len(self.occupied))
        if radius_cells <= 0:
            for index, occupied in enumerate(self.occupied):
                if occupied:
                    blocked[index] = 1
            return tuple(bool(value) for value in blocked)

        radius_sq = radius_cells * radius_cells
        row_spans = tuple(
            (dy, math.isqrt(radius_sq - dy * dy))
            for dy in range(-radius_cells, radius_cells + 1)
        )
        row_deltas = [
            [0] * (self.width + 1)
            for _ in range(self.height)
        ]
        for index, occupied in enumerate(self.occupied):
            if not occupied:
                continue
            row, col = divmod(index, self.width)
            for dy, half_width in row_spans:
                target_row = row + dy
                if not 0 <= target_row < self.height:
                    continue
                start_col = max(0, col - half_width)
                end_col = min(self.width, col + half_width + 1)
                deltas = row_deltas[target_row]
                deltas[start_col] += 1
                deltas[end_col] -= 1

        for row, deltas in enumerate(row_deltas):
            active_intervals = 0
            row_offset = row * self.width
            for col in range(self.width):
                active_intervals += deltas[col]
                if active_intervals:
                    blocked[row_offset + col] = 1
        return tuple(bool(value) for value in blocked)


class GridAStarPlanner:
    def __init__(
        self,
        grid_map: OccupancyGridMap,
        *,
        inflation_radius_m: float,
        search_padding_m: float,
        sample_spacing_m: float,
        simplify_path: bool = True,
    ) -> None:
        self._map = grid_map
        self._blocked = grid_map.inflated_blocked(inflation_radius_m)
        self._search_padding_cells = int(
            math.ceil(max(0.0, search_padding_m) / grid_map.resolution)
        )
        self._sample_spacing_m = max(0.05, sample_spacing_m)
        self._simplify_path = simplify_path

    def plan(self, waypoints: Sequence[PathPoint]) -> tuple[PathPoint, ...]:
        if len(waypoints) < 2:
            return tuple(waypoints)

        path: list[PathPoint] = []
        for start, goal in zip(waypoints, waypoints[1:]):
            cells = self._search(
                self._map.world_to_cell(start.x, start.y),
                self._map.world_to_cell(goal.x, goal.y),
            )
            if self._simplify_path:
                cells = self._simplify(cells)
            segment_path = [
                PathPoint(x=x, y=y, yaw=0.0)
                for x, y in (self._map.cell_to_world(cell) for cell in cells)
            ]
            segment_path[0] = PathPoint(
                x=start.x,
                y=start.y,
                yaw=start.yaw,
                ref_id=start.ref_id,
            )
            segment_path[-1] = PathPoint(
                x=goal.x,
                y=goal.y,
                yaw=goal.yaw,
                ref_id=goal.ref_id,
            )
            if path:
                path.extend(segment_path[1:])
            else:
                path.extend(segment_path)
        return _resample_path(
            _with_segment_yaws(path, final_yaw=waypoints[-1].yaw),
            self._sample_spacing_m,
            final_yaw=waypoints[-1].yaw,
        )

    def path_is_navigable(self, path: Sequence[PathPoint]) -> bool:
        for point in path:
            cell = self._map.world_to_cell(point.x, point.y)
            if not self._map.contains(cell) or self._is_blocked(cell):
                return False
        return True

    def _search(
        self,
        start: tuple[int, int],
        goal: tuple[int, int],
    ) -> list[tuple[int, int]]:
        self._require_navigable(start, "start")
        self._require_navigable(goal, "goal")
        bounds = self._bounds(start, goal)
        open_heap: list[tuple[float, int, tuple[int, int]]] = []
        heapq.heappush(open_heap, (0.0, 0, start))
        came_from: dict[tuple[int, int], tuple[int, int]] = {}
        g_score: dict[tuple[int, int], float] = {start: 0.0}
        closed: set[tuple[int, int]] = set()
        sequence = 0

        while open_heap:
            _, _, current = heapq.heappop(open_heap)
            if current in closed:
                continue
            if current == goal:
                return _reconstruct_path(came_from, current)
            closed.add(current)

            for neighbor, move_cost in self._neighbors(current, bounds):
                if neighbor in closed:
                    continue
                tentative = g_score[current] + move_cost
                if tentative >= g_score.get(neighbor, math.inf):
                    continue
                came_from[neighbor] = current
                g_score[neighbor] = tentative
                sequence += 1
                priority = tentative + _octile_distance(neighbor, goal)
                heapq.heappush(open_heap, (priority, sequence, neighbor))

        raise GridPlanningError(f"no grid path from {start} to {goal}")

    def _bounds(
        self,
        start: tuple[int, int],
        goal: tuple[int, int],
    ) -> tuple[int, int, int, int]:
        padding = self._search_padding_cells
        min_col = max(0, min(start[0], goal[0]) - padding)
        max_col = min(self._map.width - 1, max(start[0], goal[0]) + padding)
        min_row = max(0, min(start[1], goal[1]) - padding)
        max_row = min(self._map.height - 1, max(start[1], goal[1]) + padding)
        return min_col, max_col, min_row, max_row

    def _neighbors(
        self,
        cell: tuple[int, int],
        bounds: tuple[int, int, int, int],
    ) -> list[tuple[tuple[int, int], float]]:
        min_col, max_col, min_row, max_row = bounds
        col, row = cell
        result: list[tuple[tuple[int, int], float]] = []
        for drow in (-1, 0, 1):
            for dcol in (-1, 0, 1):
                if dcol == 0 and drow == 0:
                    continue
                neighbor = (col + dcol, row + drow)
                if not (min_col <= neighbor[0] <= max_col and min_row <= neighbor[1] <= max_row):
                    continue
                if self._is_blocked(neighbor):
                    continue
                if dcol != 0 and drow != 0:
                    if self._is_blocked((col + dcol, row)) or self._is_blocked((col, row + drow)):
                        continue
                cost = math.sqrt(2.0) if dcol != 0 and drow != 0 else 1.0
                result.append((neighbor, cost))
        return result

    def _require_navigable(self, cell: tuple[int, int], label: str) -> None:
        if not self._map.contains(cell):
            raise GridPlanningError(f"{label} cell {cell} is outside occupancy map")
        if self._is_blocked(cell):
            raise GridPlanningError(f"{label} cell {cell} is blocked after inflation")

    def _is_blocked(self, cell: tuple[int, int]) -> bool:
        return self._blocked[self._map.index(cell)]

    def _simplify(self, cells: list[tuple[int, int]]) -> list[tuple[int, int]]:
        if len(cells) <= 2:
            return cells
        simplified = [cells[0]]
        anchor = 0
        while anchor < len(cells) - 1:
            candidate = len(cells) - 1
            while candidate > anchor + 1:
                if self._line_is_clear(cells[anchor], cells[candidate]):
                    break
                candidate -= 1
            simplified.append(cells[candidate])
            anchor = candidate
        return simplified

    def _line_is_clear(
        self,
        start: tuple[int, int],
        goal: tuple[int, int],
    ) -> bool:
        for cell in _bresenham_cells(start, goal):
            if self._is_blocked(cell):
                return False
        return True


def _read_pgm(path: Path) -> tuple[int, int, list[int]]:
    with path.open("rb") as stream:
        magic = _next_pgm_token(stream)
        if magic not in (b"P5", b"P2"):
            raise GridPlanningError(f"{path} is not a PGM image")
        width = int(_next_pgm_token(stream))
        height = int(_next_pgm_token(stream))
        max_value = int(_next_pgm_token(stream))
        if max_value <= 0 or max_value > 255:
            raise GridPlanningError(f"{path} uses unsupported max value {max_value}")
        if magic == b"P5":
            data = list(stream.read(width * height))
        else:
            data = [int(_next_pgm_token(stream)) for _ in range(width * height)]
    if len(data) != width * height:
        raise GridPlanningError(f"{path} did not contain {width * height} pixels")
    if max_value != 255:
        data = [round(value * 255 / max_value) for value in data]
    return width, height, data


def _next_pgm_token(stream: Any) -> bytes:
    token = bytearray()
    while True:
        char = stream.read(1)
        if not char:
            if token:
                return bytes(token)
            raise GridPlanningError("unexpected EOF while reading PGM")
        if char == b"#":
            stream.readline()
            continue
        if char.isspace():
            if token:
                return bytes(token)
            continue
        token.extend(char)


def _reconstruct_path(
    came_from: dict[tuple[int, int], tuple[int, int]],
    current: tuple[int, int],
) -> list[tuple[int, int]]:
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path


def _octile_distance(a: tuple[int, int], b: tuple[int, int]) -> float:
    dx = abs(a[0] - b[0])
    dy = abs(a[1] - b[1])
    return dx + dy + (math.sqrt(2.0) - 2.0) * min(dx, dy)


def _bresenham_cells(
    start: tuple[int, int],
    goal: tuple[int, int],
) -> list[tuple[int, int]]:
    x0, y0 = start
    x1, y1 = goal
    dx = abs(x1 - x0)
    dy = -abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    error = dx + dy
    x, y = x0, y0
    cells = []
    while True:
        cells.append((x, y))
        if x == x1 and y == y1:
            return cells
        twice_error = 2 * error
        if twice_error >= dy:
            error += dy
            x += sx
        if twice_error <= dx:
            error += dx
            y += sy


def _with_segment_yaws(
    path: list[PathPoint],
    *,
    final_yaw: float,
) -> tuple[PathPoint, ...]:
    if len(path) < 2:
        return tuple(path)
    result = []
    for current, following in zip(path, path[1:]):
        yaw = math.atan2(following.y - current.y, following.x - current.x)
        result.append(
            PathPoint(x=current.x, y=current.y, yaw=yaw, ref_id=current.ref_id)
        )
    last = path[-1]
    result.append(PathPoint(x=last.x, y=last.y, yaw=final_yaw, ref_id=last.ref_id))
    return tuple(result)


def _resample_path(
    path: tuple[PathPoint, ...],
    spacing_m: float,
    *,
    final_yaw: float,
) -> tuple[PathPoint, ...]:
    if len(path) < 2:
        return path
    resampled: list[PathPoint] = []
    for start, end in zip(path, path[1:]):
        dx = end.x - start.x
        dy = end.y - start.y
        length = math.hypot(dx, dy)
        yaw = math.atan2(dy, dx) if length > 0.0 else start.yaw
        samples = max(1, math.ceil(length / spacing_m))
        for index in range(samples + 1):
            if resampled and index == 0:
                continue
            ratio = index / samples
            ref_id = start.ref_id if index == 0 else end.ref_id if index == samples else None
            resampled.append(
                PathPoint(
                    x=start.x + dx * ratio,
                    y=start.y + dy * ratio,
                    yaw=yaw,
                    ref_id=ref_id,
                )
            )
    last = resampled[-1]
    resampled[-1] = PathPoint(x=last.x, y=last.y, yaw=final_yaw, ref_id=last.ref_id)
    return tuple(resampled)
