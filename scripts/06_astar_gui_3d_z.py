#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import csv
import heapq
import json
import math
from pathlib import Path

import numpy as np
from PIL import Image, ImageTk, ImageDraw

import tkinter as tk
from tkinter import messagebox


def safe_float(value, default=0.0):
    try:
        if value is None:
            return default
        s = str(value).strip()
        if s == "":
            return default
        return float(s)
    except Exception:
        return default


def parse_optional_float(value):
    if value is None:
        return None
    s = str(value).strip().lower()
    if s in ["", "none", "null", "no"]:
        return None
    return float(s)


def read_heightmap_normalized(heightmap_path):
    img = Image.open(heightmap_path)
    arr = np.array(img).astype(np.float32)

    min_v = float(np.nanmin(arr))
    max_v = float(np.nanmax(arr))

    if max_v <= min_v:
        return np.zeros_like(arr, dtype=np.float32)

    norm = (arr - min_v) / (max_v - min_v)
    return np.clip(norm, 0.0, 1.0)


def load_buildings(buildings_csv):
    buildings = []

    if not Path(buildings_csv).exists():
        return buildings

    with open(buildings_csv, newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)

        for row in reader:
            x = safe_float(row.get("x_m"))
            y = safe_float(row.get("y_m"))

            width = safe_float(row.get("width_m"), 1.0)
            depth = safe_float(row.get("depth_m"), 1.0)
            height = safe_float(row.get("height_m"), 10.0)

            min_x = safe_float(row.get("min_x_m"), x - width / 2.0)
            max_x = safe_float(row.get("max_x_m"), x + width / 2.0)
            min_y = safe_float(row.get("min_y_m"), y - depth / 2.0)
            max_y = safe_float(row.get("max_y_m"), y + depth / 2.0)

            buildings.append({
                "x": x,
                "y": y,
                "width": width,
                "depth": depth,
                "height": height,
                "min_x": min_x,
                "max_x": max_x,
                "min_y": min_y,
                "max_y": max_y,
                "name": row.get("name", ""),
                "osm_id": row.get("osm_id", "")
            })

    return buildings

class GridMap3DZAstar:
    def __init__(
        self,
        map_meta,
        heightmap_path,
        buildings,
        resolution_m=5.0,
        safety_margin_m=5.0,
        terrain_clearance_m=10.0,
        building_clearance_m=10.0,
        building_base_offset_m=0.20,
        z_step_m=5.0,
        max_z_m=None,
        auto_top_margin_m=5.0,
    ):
        self.meta = map_meta
        self.heightmap_path = Path(heightmap_path)
        self.buildings = buildings

        self.size_x_m = float(map_meta["gazebo_size_x_m"])
        self.size_y_m = float(map_meta["gazebo_size_y_m"])
        self.z_size_m = float(map_meta["gazebo_z_size_m"])

        self.resolution_m = float(resolution_m)
        self.safety_margin_m = float(safety_margin_m)
        self.terrain_clearance_m = float(terrain_clearance_m)
        self.building_clearance_m = float(building_clearance_m)
        self.building_base_offset_m = float(building_base_offset_m)
        self.z_step_m = abs(float(z_step_m))
        if self.z_step_m <= 0.0:
            self.z_step_m = 5.0

        self.cols = max(2, int(math.ceil(self.size_x_m / self.resolution_m)))
        self.rows = max(2, int(math.ceil(self.size_y_m / self.resolution_m)))

        if self.cols > 1500 or self.rows > 1500:
            raise RuntimeError(
                f"Grid가 너무 큽니다: {self.cols} x {self.rows}\n"
                f"--resolution 값을 더 크게 주세요. 예: --resolution 10"
            )

        self.height_norm = read_heightmap_normalized(self.heightmap_path)

        self.precompute_building_altitudes()

        self.required_z_grid = None
        self.terrain_z_grid = None
        self.building_top_grid = None
        self.required_reason_grid = None
        self.build_required_z_grid()

        min_required = float(np.nanmin(self.required_z_grid))
        max_required = float(np.nanmax(self.required_z_grid))

        self.min_z_m = math.floor(min_required / self.z_step_m) * self.z_step_m
        auto_max_z = math.ceil((max_required + float(auto_top_margin_m)) / self.z_step_m) * self.z_step_m

        if max_z_m is None:
            self.max_z_m = auto_max_z
        else:
            self.max_z_m = float(max_z_m)

        if self.max_z_m < max_required:
            raise RuntimeError(
                f"--max-z가 너무 낮습니다.\n"
                f"필요한 최소 최대 z = {max_required:.3f} m, 현재 max-z = {self.max_z_m:.3f} m"
            )

        self.z_layers = int(math.floor((self.max_z_m - self.min_z_m) / self.z_step_m)) + 1
        if self.z_layers < 1:
            raise RuntimeError("z layer 개수가 1보다 작습니다. --max-z 또는 --z-step을 확인하세요.")

        total_states = self.rows * self.cols * self.z_layers
        if total_states > 8_000_000:
            raise RuntimeError(
                f"3D A* 상태공간이 너무 큽니다: rows={self.rows}, cols={self.cols}, z_layers={self.z_layers}, "
                f"states={total_states:,}\n"
                f"해결: --resolution 값을 키우거나, --z-step 값을 키우거나, --max-z를 낮추세요."
            )

    def world_to_grid(self, x, y):
        col = int((x + self.size_x_m / 2.0) / self.size_x_m * self.cols)
        row = int((self.size_y_m / 2.0 - y) / self.size_y_m * self.rows)

        col = max(0, min(self.cols - 1, col))
        row = max(0, min(self.rows - 1, row))

        return row, col

    def grid_to_world(self, row, col):
        x = ((col + 0.5) / self.cols) * self.size_x_m - self.size_x_m / 2.0
        y = self.size_y_m / 2.0 - ((row + 0.5) / self.rows) * self.size_y_m
        return x, y

    def z_to_layer(self, z):
        k = int(math.ceil((float(z) - self.min_z_m) / self.z_step_m - 1e-9))
        return max(0, min(self.z_layers - 1, k))

    def layer_to_z(self, k):
        return self.min_z_m + int(k) * self.z_step_m

    def world_to_height_pixel(self, x, y):
        h, w = self.height_norm.shape

        px = (x + self.size_x_m / 2.0) / self.size_x_m * (w - 1)
        py = (self.size_y_m / 2.0 - y) / self.size_y_m * (h - 1)

        px = int(round(max(0, min(w - 1, px))))
        py = int(round(max(0, min(h - 1, py))))

        return py, px

    def world_to_height_pixel_float(self, x, y):
        h, w = self.height_norm.shape

        px = (x + self.size_x_m / 2.0) / self.size_x_m * (w - 1)
        py = (self.size_y_m / 2.0 - y) / self.size_y_m * (h - 1)

        return py, px

    def terrain_z(self, x, y):
        py, px = self.world_to_height_pixel(x, y)
        return float(self.height_norm[py, px]) * self.z_size_m

    def terrain_max_under_rect(self, min_x, max_x, min_y, max_y, padding_m=2.0):
        h, w = self.height_norm.shape

        min_x = min_x - padding_m
        max_x = max_x + padding_m
        min_y = min_y - padding_m
        max_y = max_y + padding_m

        py1, px1 = self.world_to_height_pixel_float(min_x, max_y)
        py2, px2 = self.world_to_height_pixel_float(max_x, min_y)

        r0 = max(0, min(h - 1, int(math.floor(min(py1, py2)))))
        r1 = max(0, min(h - 1, int(math.ceil(max(py1, py2)))))
        c0 = max(0, min(w - 1, int(math.floor(min(px1, px2)))))
        c1 = max(0, min(w - 1, int(math.ceil(max(px1, px2)))))

        patch = self.height_norm[r0:r1 + 1, c0:c1 + 1]

        if patch.size == 0:
            cx = (min_x + max_x) / 2.0
            cy = (min_y + max_y) / 2.0
            return self.terrain_z(cx, cy)

        return float(np.max(patch)) * self.z_size_m

    def precompute_building_altitudes(self):
        for b in self.buildings:
            base_z = self.terrain_max_under_rect(
                min_x=b["min_x"],
                max_x=b["max_x"],
                min_y=b["min_y"],
                max_y=b["max_y"],
                padding_m=2.0
            ) + self.building_base_offset_m

            b["base_z"] = base_z
            b["top_z"] = base_z + float(b.get("height", 0.0))

    def build_required_z_grid(self):
        h, w = self.height_norm.shape

        cols = np.arange(self.cols, dtype=np.float64)
        rows = np.arange(self.rows, dtype=np.float64)

        xs = ((cols + 0.5) / self.cols) * self.size_x_m - self.size_x_m / 2.0
        ys = self.size_y_m / 2.0 - ((rows + 0.5) / self.rows) * self.size_y_m

        px = np.round((xs + self.size_x_m / 2.0) / self.size_x_m * (w - 1)).astype(np.int64)
        py = np.round((self.size_y_m / 2.0 - ys) / self.size_y_m * (h - 1)).astype(np.int64)
        px = np.clip(px, 0, w - 1)
        py = np.clip(py, 0, h - 1)

        terrain_grid = self.height_norm[py[:, None], px[None, :]].astype(np.float64) * self.z_size_m
        required = terrain_grid + self.terrain_clearance_m

        building_top = np.full((self.rows, self.cols), np.nan, dtype=np.float64)
        reason = np.full((self.rows, self.cols), "terrain_clearance", dtype=object)

        margin = self.safety_margin_m

        for b in self.buildings:
            min_x = float(b["min_x"]) - margin
            max_x = float(b["max_x"]) + margin
            min_y = float(b["min_y"]) - margin
            max_y = float(b["max_y"]) + margin

            r1, c1 = self.world_to_grid(min_x, max_y)
            r2, c2 = self.world_to_grid(max_x, min_y)

            r0 = max(0, min(r1, r2))
            r3 = min(self.rows - 1, max(r1, r2))
            c0 = max(0, min(c1, c2))
            c3 = min(self.cols - 1, max(c1, c2))

            if r0 > r3 or c0 > c3:
                continue

            top_z = float(b.get("top_z", 0.0))
            req_z = top_z + self.building_clearance_m

            patch = required[r0:r3 + 1, c0:c3 + 1]
            update_mask = req_z > patch

            if np.any(update_mask):
                patch[update_mask] = req_z
                required[r0:r3 + 1, c0:c3 + 1] = patch

                top_patch = building_top[r0:r3 + 1, c0:c3 + 1]
                top_patch[update_mask] = top_z
                building_top[r0:r3 + 1, c0:c3 + 1] = top_patch

                reason_patch = reason[r0:r3 + 1, c0:c3 + 1]
                reason_patch[update_mask] = "building_overflight"
                reason[r0:r3 + 1, c0:c3 + 1] = reason_patch

        self.terrain_z_grid = terrain_grid
        self.required_z_grid = required
        self.building_top_grid = building_top
        self.required_reason_grid = reason

    def required_z_at_cell(self, row, col):
        return float(self.required_z_grid[row, col])

    def terrain_z_at_cell(self, row, col):
        return float(self.terrain_z_grid[row, col])

    def building_top_at_cell(self, row, col):
        v = float(self.building_top_grid[row, col])
        if math.isnan(v):
            return None
        return v

    def reason_at_cell(self, row, col):
        return str(self.required_reason_grid[row, col])

    def min_valid_layer(self, row, col):
        required_z = self.required_z_at_cell(row, col)
        k = self.z_to_layer(required_z)
        if k < 0 or k >= self.z_layers:
            return None
        if self.layer_to_z(k) + 1e-9 < required_z:
            k += 1
        if k >= self.z_layers:
            return None
        return k

    def is_valid_state(self, row, col, layer):
        if row < 0 or row >= self.rows or col < 0 or col >= self.cols:
            return False
        if layer < 0 or layer >= self.z_layers:
            return False
        z = self.layer_to_z(layer)
        return z + 1e-9 >= self.required_z_at_cell(row, col)

    def state_to_point(self, state, index=0):
        row, col, layer = state
        x, y = self.grid_to_world(row, col)
        z = self.layer_to_z(layer)
        building_top = self.building_top_at_cell(row, col)
        reason = self.reason_at_cell(row, col)

        if z > self.required_z_at_cell(row, col) + self.z_step_m * 0.5:
            reason = "higher_layer_for_astar_transition"

        return {
            "index": index,
            "grid_row": row,
            "grid_col": col,
            "z_layer": layer,
            "gazebo_x_m": x,
            "gazebo_y_m": y,
            "gazebo_z_m": z,
            "terrain_z_m": self.terrain_z_at_cell(row, col),
            "required_z_m": self.required_z_at_cell(row, col),
            "terrain_clearance_m": self.terrain_clearance_m,
            "nearby_building_top_z_m": building_top,
            "building_clearance_m": self.building_clearance_m,
            "planner_mode": "3d_z_astar",
            "altitude_reason": reason,
        }


def astar_3d_z(grid_map, start_cell, goal_cell, z_cost_weight=1.0, max_expanded=700000):

    sr, sc = start_cell
    gr, gc = goal_cell

    sk = grid_map.min_valid_layer(sr, sc)
    gk = grid_map.min_valid_layer(gr, gc)

    if sk is None:
        raise RuntimeError("START 위치에서 필요한 고도를 만족하는 z layer가 없습니다. --max-z를 올리세요.")
    if gk is None:
        raise RuntimeError("GOAL 위치에서 필요한 고도를 만족하는 z layer가 없습니다. --max-z를 올리세요.")

    start = (sr, sc, sk)
    goal = (gr, gc, gk)

    z_cost_weight = float(z_cost_weight)
    if z_cost_weight < 0.0:
        z_cost_weight = 0.0

    neighbors_3d = []

    for dr in (-1, 0, 1):
        for dc in (-1, 0, 1):
            for dk in (-1, 0, 1):
                if dr == 0 and dc == 0 and dk == 0:
                    continue

                dx_m = dc * grid_map.resolution_m
                dy_m = dr * grid_map.resolution_m
                dz_m = dk * grid_map.z_step_m * z_cost_weight

                step_cost = math.sqrt(dx_m * dx_m + dy_m * dy_m + dz_m * dz_m)

                if step_cost <= 0.0:
                    step_cost = 1e-6

                neighbors_3d.append((dr, dc, dk, step_cost))

    def heuristic(r, c, k):

        dx = abs(c - gc) * grid_map.resolution_m
        dy = abs(r - gr) * grid_map.resolution_m
        dz = abs(grid_map.layer_to_z(k) - grid_map.layer_to_z(gk)) * z_cost_weight
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def diagonal_xy_is_safe(r, c, nr, nc, nk):

        dr = nr - r
        dc = nc - c

        if dr != 0 and dc != 0:
            if not grid_map.is_valid_state(r, nc, nk):
                return False
            if not grid_map.is_valid_state(nr, c, nk):
                return False

        return True

    def vertical_transition_is_safe(r, c, k, nk):

        if nk == k:
            return True
        return grid_map.is_valid_state(r, c, nk)

    open_heap = []
    counter = 0
    heapq.heappush(open_heap, (heuristic(*start), counter, 0.0, start))

    came_from = {}
    g_score = {start: 0.0}
    visited = set()
    expanded = 0

    while open_heap:
        _, _, current_g, current = heapq.heappop(open_heap)

        if current in visited:
            continue

        visited.add(current)
        expanded += 1

        if expanded > max_expanded:
            raise RuntimeError(
                f"A* 탐색량이 너무 큽니다. expanded={expanded:,}\n"
                f"해결: --resolution을 키우거나, --z-step을 키우거나, --max-expanded를 올리세요."
            )

        r, c, k = current

        if current == goal:
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            path.reverse()
            return path, expanded

        for dr, dc, dk, step_cost in neighbors_3d:
            nr = r + dr
            nc = c + dc
            nk = k + dk

            if not grid_map.is_valid_state(nr, nc, nk):
                continue

            if not vertical_transition_is_safe(r, c, k, nk):
                continue

            if not diagonal_xy_is_safe(r, c, nr, nc, nk):
                continue

            nxt = (nr, nc, nk)
            tentative_g = current_g + step_cost

            if tentative_g < g_score.get(nxt, float("inf")):
                came_from[nxt] = current
                g_score[nxt] = tentative_g
                counter += 1
                f = tentative_g + heuristic(nr, nc, nk)
                heapq.heappush(open_heap, (f, counter, tentative_g, nxt))

    return None, expanded


def states_to_points(grid_map, states):
    points = []
    for i, state in enumerate(states):
        points.append(grid_map.state_to_point(state, index=i))
    return points


def simplify_3d_points(points, z_threshold_m=0.1, max_segment_m=80.0):
    if points is None or len(points) <= 2:
        return points

    result = [points[0]]

    prev_dir = None
    last_keep = points[0]

    for i in range(1, len(points) - 1):
        a = points[i - 1]
        b = points[i]

        dr = int(b["grid_row"]) - int(a["grid_row"])
        dc = int(b["grid_col"]) - int(a["grid_col"])

        try:
            dk_raw = int(b.get("z_layer", "")) - int(a.get("z_layer", ""))
        except Exception:
            dk_raw = float(b["gazebo_z_m"]) - float(a["gazebo_z_m"])

        dr = int(math.copysign(1, dr)) if dr != 0 else 0
        dc = int(math.copysign(1, dc)) if dc != 0 else 0
        dk = int(math.copysign(1, dk_raw)) if dk_raw != 0 else 0
        cur_dir = (dr, dc, dk)

        dx = float(b["gazebo_x_m"]) - float(last_keep["gazebo_x_m"])
        dy = float(b["gazebo_y_m"]) - float(last_keep["gazebo_y_m"])
        dz = abs(float(b["gazebo_z_m"]) - float(last_keep["gazebo_z_m"]))
        dxy = math.sqrt(dx * dx + dy * dy)

        keep = False
        if prev_dir is None:
            prev_dir = cur_dir
        elif cur_dir != prev_dir:
            keep = True
            prev_dir = cur_dir

        if dz >= z_threshold_m:
            keep = True

        if dxy >= max_segment_m:
            keep = True

        if keep:
            result.append(points[i])
            last_keep = points[i]

    result.append(points[-1])

    for i, p in enumerate(result):
        p["index"] = i

    return result


def append_landing_descent(
    grid_map,
    points,
    add_landing=True,
    landing_final_clearance_m=0.5,
    landing_step_z_m=1.0
):

    result = [dict(p) for p in points]

    if not add_landing or not result:
        return result

    step = abs(float(landing_step_z_m))
    if step <= 0.0:
        step = 1.0

    last = result[-1].copy()

    x = float(last["gazebo_x_m"])
    y = float(last["gazebo_y_m"])
    row, col = grid_map.world_to_grid(x, y)

    terrain_z = grid_map.terrain_z_at_cell(row, col)
    final_z = terrain_z + float(landing_final_clearance_m)
    start_z = float(last["gazebo_z_m"])

    if start_z <= final_z:
        return result

    descent_count = int(math.ceil((start_z - final_z) / step))

    for k in range(1, descent_count + 1):
        z = max(final_z, start_z - step * k)

        landing_point = last.copy()
        landing_point["index"] = len(result)
        landing_point["grid_row"] = row
        landing_point["grid_col"] = col
        landing_point["z_layer"] = ""
        landing_point["gazebo_x_m"] = x
        landing_point["gazebo_y_m"] = y
        landing_point["gazebo_z_m"] = z
        landing_point["terrain_z_m"] = terrain_z
        landing_point["required_z_m"] = final_z
        landing_point["terrain_clearance_m"] = float(landing_final_clearance_m)
        landing_point["nearby_building_top_z_m"] = ""
        landing_point["building_clearance_m"] = float(last.get("building_clearance_m", 0.0))
        landing_point["planner_mode"] = "3d_z_astar_with_landing"
        landing_point["altitude_reason"] = "landing_descent"

        result.append(landing_point)

    return result


def write_path_csv(points, output_csv):
    output_csv = Path(output_csv)
    output_csv.parent.mkdir(parents=True, exist_ok=True)

    fieldnames = [
        "index",
        "grid_row",
        "grid_col",
        "z_layer",
        "gazebo_x_m",
        "gazebo_y_m",
        "gazebo_z_m",
        "terrain_z_m",
        "required_z_m",
        "terrain_clearance_m",
        "nearby_building_top_z_m",
        "building_clearance_m",
        "planner_mode",
        "altitude_reason",
    ]

    with open(output_csv, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()

        for i, p in enumerate(points):
            building_top = p.get("nearby_building_top_z_m")
            if building_top is None or building_top == "":
                building_top_text = ""
            else:
                building_top_text = f"{float(building_top):.6f}"

            z_layer = p.get("z_layer", "")
            z_layer_text = "" if z_layer == "" else str(int(z_layer))

            writer.writerow({
                "index": i,
                "grid_row": int(p["grid_row"]),
                "grid_col": int(p["grid_col"]),
                "z_layer": z_layer_text,
                "gazebo_x_m": f"{float(p['gazebo_x_m']):.6f}",
                "gazebo_y_m": f"{float(p['gazebo_y_m']):.6f}",
                "gazebo_z_m": f"{float(p['gazebo_z_m']):.6f}",
                "terrain_z_m": f"{float(p['terrain_z_m']):.6f}",
                "required_z_m": f"{float(p.get('required_z_m', p['terrain_z_m'])):.6f}",
                "terrain_clearance_m": f"{float(p['terrain_clearance_m']):.6f}",
                "nearby_building_top_z_m": building_top_text,
                "building_clearance_m": f"{float(p['building_clearance_m']):.6f}",
                "planner_mode": p.get("planner_mode", "3d_z_astar"),
                "altitude_reason": p.get("altitude_reason", ""),
            })

class AstarGUI3DZ:
    def __init__(
        self,
        root,
        grid_map,
        preview_path,
        output_csv,
        output_full_csv,
        output_preview,
        z_cost_weight=1.0,
        max_expanded=700000,
        simplify_z_threshold=0.1,
        simplify_max_segment=80.0,
        add_landing=True,
        landing_final_clearance_m=0.5,
        landing_step_z_m=1.0,
    ):
        self.root = root
        self.grid_map = grid_map
        self.preview_path = Path(preview_path)
        self.output_csv = Path(output_csv)
        self.output_full_csv = Path(output_full_csv)
        self.z_cost_weight = float(z_cost_weight)
        self.max_expanded = int(max_expanded)
        self.simplify_z_threshold = float(simplify_z_threshold)
        self.simplify_max_segment = float(simplify_max_segment)
        self.add_landing = bool(add_landing)
        self.landing_final_clearance_m = float(landing_final_clearance_m)
        self.landing_step_z_m = float(landing_step_z_m)

        if output_preview is None:
            self.output_preview = self.output_full_csv.parent / "path_preview_3d_z_astar.png"
        else:
            self.output_preview = Path(output_preview)

        self.start_cell = None
        self.goal_cell = None
        self.path_states = None
        self.path_points = None
        self.path_simplified = None

        self.max_display_w = 1100
        self.max_display_h = 850

        self.base_img = self.make_base_image()
        self.display_img = self.base_img.copy()

        self.img_w, self.img_h = self.display_img.size

        self.root.title("3D-Z A* - Buildings are Altitude Constraints")

        self.canvas = tk.Canvas(
            self.root,
            width=self.img_w,
            height=self.img_h,
            bg="white"
        )
        self.canvas.pack(side=tk.TOP, padx=10, pady=10)

        self.tk_img = ImageTk.PhotoImage(self.display_img)
        self.canvas_img_id = self.canvas.create_image(
            0,
            0,
            anchor=tk.NW,
            image=self.tk_img
        )

        self.canvas.bind("<Button-1>", self.on_left_click)

        control = tk.Frame(self.root)
        control.pack(side=tk.BOTTOM, fill=tk.X, padx=10, pady=6)

        self.info_label = tk.Label(
            control,
            text=self.status_text(),
            anchor="w",
            justify="left"
        )
        self.info_label.pack(side=tk.LEFT, padx=5)

        btn_reset = tk.Button(control, text="Reset", command=self.reset)
        btn_reset.pack(side=tk.RIGHT, padx=5)

        btn_save = tk.Button(control, text="Save Path Again", command=self.save_current_path)
        btn_save.pack(side=tk.RIGHT, padx=5)

        self.root.bind("r", lambda e: self.reset())
        self.root.bind("R", lambda e: self.reset())

    def make_base_image(self):
        img = Image.open(self.preview_path).convert("RGB")

        try:
            resample_filter = Image.Resampling.LANCZOS
        except AttributeError:
            resample_filter = Image.LANCZOS

        img.thumbnail((self.max_display_w, self.max_display_h), resample_filter)
        img = img.convert("RGBA")

        overlay = Image.new("RGBA", img.size, (0, 0, 0, 0))
        draw = ImageDraw.Draw(overlay)

        for b in self.grid_map.buildings:
            x1, y1 = self.world_to_image(b["min_x"], b["max_y"], img.size)
            x2, y2 = self.world_to_image(b["max_x"], b["min_y"], img.size)

            draw.rectangle(
                [x1, y1, x2, y2],
                fill=(255, 120, 0, 75),
                outline=(180, 70, 0, 170),
                width=1
            )

            m = self.grid_map.safety_margin_m
            sx1, sy1 = self.world_to_image(b["min_x"] - m, b["max_y"] + m, img.size)
            sx2, sy2 = self.world_to_image(b["max_x"] + m, b["min_y"] - m, img.size)
            draw.rectangle(
                [sx1, sy1, sx2, sy2],
                outline=(255, 160, 0, 90),
                width=1
            )

        img = Image.alpha_composite(img, overlay)
        return img.convert("RGB")

    def world_to_image(self, x, y, size=None):
        if size is None:
            w, h = self.img_w, self.img_h
        else:
            w, h = size

        px = (x + self.grid_map.size_x_m / 2.0) / self.grid_map.size_x_m * (w - 1)
        py = (self.grid_map.size_y_m / 2.0 - y) / self.grid_map.size_y_m * (h - 1)

        return int(round(px)), int(round(py))

    def image_to_world(self, px, py):
        x = px / max(1, self.img_w - 1) * self.grid_map.size_x_m - self.grid_map.size_x_m / 2.0
        y = self.grid_map.size_y_m / 2.0 - py / max(1, self.img_h - 1) * self.grid_map.size_y_m
        return x, y

    def status_text(self):
        return (
            "사용법: 왼쪽 클릭 1번 = 출발점, 왼쪽 클릭 2번 = 도착점 / R = Reset\n"
            f"Mode: 3D-Z A*, Buildings = Altitude Constraints, "
            f"Grid: {self.grid_map.cols} x {self.grid_map.rows} x {self.grid_map.z_layers}, "
            f"Resolution: {self.grid_map.resolution_m:.2f} m, "
            f"Z step: {self.grid_map.z_step_m:.2f} m, "
            f"Z range: {self.grid_map.min_z_m:.2f}~{self.grid_map.max_z_m:.2f} m, "
            f"Z cost weight: {self.z_cost_weight:.2f}, "
            f"Landing: {'ON' if self.add_landing else 'OFF'} -> terrain+{self.landing_final_clearance_m:.2f} m"
        )

    def refresh_image(self):
        img = self.base_img.copy().convert("RGBA")
        draw = ImageDraw.Draw(img)

        if self.start_cell is not None:
            x, y = self.grid_map.grid_to_world(*self.start_cell)
            px, py = self.world_to_image(x, y)
            r = 7
            draw.ellipse([px - r, py - r, px + r, py + r], fill=(0, 255, 0, 255), outline=(0, 80, 0, 255), width=2)
            draw.text((px + 10, py - 10), "START", fill=(0, 120, 0, 255))

        if self.goal_cell is not None:
            x, y = self.grid_map.grid_to_world(*self.goal_cell)
            px, py = self.world_to_image(x, y)
            r = 7
            draw.ellipse([px - r, py - r, px + r, py + r], fill=(0, 80, 255, 255), outline=(0, 0, 120, 255), width=2)
            draw.text((px + 10, py - 10), "GOAL", fill=(0, 0, 180, 255))

        if self.path_points is not None:
            pts = []
            for p in self.path_points:
                pts.append(self.world_to_image(float(p["gazebo_x_m"]), float(p["gazebo_y_m"])))

            if len(pts) >= 2:
                draw.line(pts, fill=(255, 255, 0, 230), width=5)
                draw.line(pts, fill=(255, 0, 0, 255), width=2)

        if self.path_simplified is not None:
            for p in self.path_simplified:
                px, py = self.world_to_image(float(p["gazebo_x_m"]), float(p["gazebo_y_m"]))
                r = 3
                z = float(p["gazebo_z_m"])
                draw.ellipse([px - r, py - r, px + r, py + r], fill=(255, 255, 255, 255), outline=(0, 0, 0, 255))
                draw.text((px + 4, py + 4), f"z={z:.0f}", fill=(120, 0, 0, 255))

        self.display_img = img.convert("RGB")
        self.tk_img = ImageTk.PhotoImage(self.display_img)
        self.canvas.itemconfig(self.canvas_img_id, image=self.tk_img)

    def on_left_click(self, event):
        x, y = self.image_to_world(event.x, event.y)
        row, col = self.grid_map.world_to_grid(x, y)

        min_layer = self.grid_map.min_valid_layer(row, col)
        if min_layer is None:
            messagebox.showerror("Error", "클릭한 지점에서 가능한 z layer가 없습니다. --max-z를 올리세요.")
            return

        if self.start_cell is None:
            self.start_cell = (row, col)
            self.goal_cell = None
            self.path_states = None
            self.path_points = None
            self.path_simplified = None
            wx, wy = self.grid_map.grid_to_world(row, col)
            print(
                "START:", self.start_cell,
                (wx, wy),
                "required_z=", f"{self.grid_map.required_z_at_cell(row, col):.3f}",
                "start_z=", f"{self.grid_map.layer_to_z(min_layer):.3f}"
            )

        elif self.goal_cell is None:
            self.goal_cell = (row, col)
            wx, wy = self.grid_map.grid_to_world(row, col)
            print(
                "GOAL :", self.goal_cell,
                (wx, wy),
                "required_z=", f"{self.grid_map.required_z_at_cell(row, col):.3f}",
                "goal_z=", f"{self.grid_map.layer_to_z(min_layer):.3f}"
            )
            self.run_astar()

        else:
            self.start_cell = (row, col)
            self.goal_cell = None
            self.path_states = None
            self.path_points = None
            self.path_simplified = None
            wx, wy = self.grid_map.grid_to_world(row, col)
            print(
                "새 START:", self.start_cell,
                (wx, wy),
                "required_z=", f"{self.grid_map.required_z_at_cell(row, col):.3f}",
                "start_z=", f"{self.grid_map.layer_to_z(min_layer):.3f}"
            )

        self.refresh_image()

    def run_astar(self):
        if self.start_cell is None or self.goal_cell is None:
            return

        print("")
        print("========== 3D-Z A* 경로 계산 시작 ==========")
        print("start:", self.start_cell)
        print("goal :", self.goal_cell)
        print("grid :", f"{self.grid_map.cols} x {self.grid_map.rows} x {self.grid_map.z_layers}")
        print("z range:", f"{self.grid_map.min_z_m:.3f} ~ {self.grid_map.max_z_m:.3f} m")
        print("z step:", f"{self.grid_map.z_step_m:.3f} m")
        print("z cost weight:", self.z_cost_weight)

        try:
            states, expanded = astar_3d_z(
                grid_map=self.grid_map,
                start_cell=self.start_cell,
                goal_cell=self.goal_cell,
                z_cost_weight=self.z_cost_weight,
                max_expanded=self.max_expanded,
            )

            if states is None:
                raise RuntimeError("경로를 찾지 못했습니다. --max-z, --resolution, --z-step 값을 조정하세요.")

            base_points = states_to_points(self.grid_map, states)
            full_points = append_landing_descent(
                grid_map=self.grid_map,
                points=base_points,
                add_landing=self.add_landing,
                landing_final_clearance_m=self.landing_final_clearance_m,
                landing_step_z_m=self.landing_step_z_m,
            )

        except Exception as e:
            self.path_states = None
            self.path_points = None
            self.path_simplified = None
            messagebox.showerror("3D-Z A* 실패", str(e))
            print("[FAIL]", e)
            return

        simplified = simplify_3d_points(
            full_points,
            z_threshold_m=self.simplify_z_threshold,
            max_segment_m=self.simplify_max_segment,
        )

        self.path_states = states
        self.path_points = full_points
        self.path_simplified = simplified

        write_path_csv(full_points, self.output_full_csv)
        write_path_csv(simplified, self.output_csv)

        z_values = [float(p["gazebo_z_m"]) for p in full_points]
        terrain_values = [float(p["terrain_z_m"]) for p in full_points]
        overflight_count = sum(1 for p in full_points if "building" in str(p["altitude_reason"]))

        length_xy = 0.0
        climb_total = 0.0
        for a, b in zip(full_points[:-1], full_points[1:]):
            dx = float(b["gazebo_x_m"]) - float(a["gazebo_x_m"])
            dy = float(b["gazebo_y_m"]) - float(a["gazebo_y_m"])
            dz = float(b["gazebo_z_m"]) - float(a["gazebo_z_m"])
            length_xy += math.sqrt(dx * dx + dy * dy)
            if dz > 0:
                climb_total += dz

        print("A* expanded states   :", f"{expanded:,}")
        print("A* state path 개수   :", len(states))
        print("full waypoint 개수   :", len(full_points))
        print("간소화 waypoint 개수 :", len(simplified))
        print("XY path length       :", f"{length_xy:.3f} m")
        print("total climb          :", f"{climb_total:.3f} m")
        print("terrain z min/max    :", f"{min(terrain_values):.3f} / {max(terrain_values):.3f}")
        print("flight z min/max     :", f"{min(z_values):.3f} / {max(z_values):.3f}")
        print("building 관련 point 수:", overflight_count)
        print("착륙 waypoint 추가   :", self.add_landing)
        if self.add_landing:
            print("착륙 최종 gazebo z   :", f"{float(full_points[-1]['gazebo_z_m']):.3f} m")
        print("비행용 full path 저장:", self.output_full_csv)
        print("간소화 path 저장     :", self.output_csv)
        print("")

        self.refresh_image()
        self.output_preview.parent.mkdir(parents=True, exist_ok=True)
        self.display_img.save(self.output_preview)

        messagebox.showinfo(
            "3D-Z A* 완료",
            f"경로 생성 완료!\n\n"
            f"A* expanded states: {expanded:,}\n"
            f"A* state path 개수: {len(states)}\n"
            f"전체 waypoint 개수: {len(full_points)}\n"
            f"간소화 waypoint 개수: {len(simplified)}\n"
            f"XY path length: {length_xy:.2f} m\n"
            f"z min/max: {min(z_values):.2f} / {max(z_values):.2f} m\n"
            f"건물/고도상승 관련 point 수: {overflight_count}\n"
            f"착륙 waypoint: {'ON' if self.add_landing else 'OFF'}\n\n"
            f"실제 비행 권장 CSV:\n{self.output_full_csv}\n\n"
            f"간소화 확인용 CSV:\n{self.output_csv}"
        )

    def save_current_path(self):
        if self.path_points is None or self.path_simplified is None:
            messagebox.showwarning("No path", "아직 생성된 경로가 없습니다.")
            return

        write_path_csv(self.path_points, self.output_full_csv)
        write_path_csv(self.path_simplified, self.output_csv)

        self.refresh_image()
        self.output_preview.parent.mkdir(parents=True, exist_ok=True)
        self.display_img.save(self.output_preview)

        messagebox.showinfo(
            "저장 완료",
            f"비행용 full path를 다시 저장했습니다:\n{self.output_full_csv}\n\n"
            f"간소화 path도 다시 저장했습니다:\n{self.output_csv}"
        )

    def reset(self):
        self.start_cell = None
        self.goal_cell = None
        self.path_states = None
        self.path_points = None
        self.path_simplified = None
        self.refresh_image()
        print("Reset 완료")

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--meta", default="../output/map_meta.json")
    parser.add_argument("--heightmap", default="../output/heightmap.png")
    parser.add_argument("--preview", default="../output/heightmap_preview.png")
    parser.add_argument("--buildings", default="../output/buildings.csv")
    parser.add_argument(
        "--output",
        default="../output/path_full_dynamic_z.csv",
        help="드론 비행용 전체 경로 CSV"
    )
    parser.add_argument(
        "--output-simple",
        default="../output/path_dynamic_z.csv",
        help="확인/시각화용 간소화 경로 CSV"
    )
    parser.add_argument(
        "--output-preview",
        default=None,
        help="경로 미리보기 이미지 저장 경로"
    )
    parser.add_argument("--resolution", type=float, default=5.0, help="A* XY grid resolution in meters")
    parser.add_argument("--safety-margin", type=float, default=5.0, help="건물 footprint 주변 안전거리")
    parser.add_argument("--terrain-clearance", type=float, default=10.0, help="지형보다 몇 m 위로 날지")
    parser.add_argument("--building-clearance", type=float, default=10.0, help="건물 꼭대기보다 몇 m 위로 날지")
    parser.add_argument("--building-base-offset", type=float, default=0.20, help="05번 world 생성과 맞춘 건물 바닥 보정값")
    parser.add_argument("--z-step", type=float, default=5.0, help="3D A* z layer 간격. 작을수록 정밀하지만 느림")
    parser.add_argument("--max-z", default=None, help="최대 허용 z. None이면 필요한 최대 z 기준 자동 계산")
    parser.add_argument("--auto-top-margin", type=float, default=5.0, help="max-z 자동 계산 시 필요한 최대 z 위에 더할 여유")
    parser.add_argument("--z-cost-weight", type=float, default=1.0, help="상승/하강 비용 가중치. 낮으면 건물 위로 넘고, 높으면 돌아가는 경향")
    parser.add_argument("--max-expanded", type=int, default=700000, help="A* 최대 확장 state 수")
    parser.add_argument("--simplify-z-threshold", type=float, default=0.1, help="간소화 CSV에서 유지할 최소 z 변화량")
    parser.add_argument("--simplify-max-segment", type=float, default=80.0, help="간소화 CSV에서 waypoint 사이 최대 XY 거리")

    parser.add_argument(
        "--add-landing",
        dest="add_landing",
        action="store_true",
        default=True,
        help="경로 끝에 같은 XY에서 z를 낮추는 착륙 waypoint를 추가"
    )
    parser.add_argument(
        "--no-add-landing",
        dest="add_landing",
        action="store_false",
        help="착륙 waypoint 추가를 끔"
    )
    parser.add_argument(
        "--landing-final-clearance",
        type=float,
        default=0.5,
        help="3D-Z 착륙 최종 고도. goal 지형 높이 + 이 값"
    )
    parser.add_argument(
        "--landing-step-z",
        type=float,
        default=1.0,
        help="착륙 시 waypoint마다 낮출 z 간격"
    )

    args = parser.parse_args()

    meta_path = Path(args.meta)
    heightmap_path = Path(args.heightmap)
    preview_path = Path(args.preview)
    buildings_path = Path(args.buildings)
    max_z_m = parse_optional_float(args.max_z)

    if not meta_path.exists():
        raise FileNotFoundError(f"map_meta.json이 없습니다: {meta_path}")

    if not heightmap_path.exists():
        raise FileNotFoundError(f"heightmap.png가 없습니다: {heightmap_path}")

    if not preview_path.exists():
        raise FileNotFoundError(f"heightmap_preview.png가 없습니다: {preview_path}")

    if not buildings_path.exists():
        raise FileNotFoundError(f"buildings.csv가 없습니다: {buildings_path}")

    with open(meta_path, "r", encoding="utf-8") as f:
        meta = json.load(f)

    buildings = load_buildings(buildings_path)

    print("")
    print("========== 3D-Z A* GUI 로딩 ==========")
    print(f"meta               : {meta_path}")
    print(f"heightmap          : {heightmap_path}")
    print(f"preview            : {preview_path}")
    print(f"buildings          : {buildings_path}")
    print(f"building count     : {len(buildings)}")
    print(f"resolution         : {args.resolution} m")
    print(f"safety margin      : {args.safety_margin} m")
    print(f"terrain clearance  : {args.terrain_clearance} m")
    print(f"building clearance : {args.building_clearance} m")
    print(f"z step             : {args.z_step} m")
    print(f"z cost weight      : {args.z_cost_weight}")
    print(f"max z              : {max_z_m}")
    print(f"add landing        : {args.add_landing}")
    print(f"landing final clr  : {args.landing_final_clearance} m")
    print(f"landing step z     : {args.landing_step_z} m")
    print("")

    grid_map = GridMap3DZAstar(
        map_meta=meta,
        heightmap_path=heightmap_path,
        buildings=buildings,
        resolution_m=args.resolution,
        safety_margin_m=args.safety_margin,
        terrain_clearance_m=args.terrain_clearance,
        building_clearance_m=args.building_clearance,
        building_base_offset_m=args.building_base_offset,
        z_step_m=args.z_step,
        max_z_m=max_z_m,
        auto_top_margin_m=args.auto_top_margin,
    )

    print("Grid size:", grid_map.cols, "x", grid_map.rows, "x", grid_map.z_layers)
    print("Z range  :", f"{grid_map.min_z_m:.3f} ~ {grid_map.max_z_m:.3f} m")
    print("Required z min/max:", f"{float(np.nanmin(grid_map.required_z_grid)):.3f} / {float(np.nanmax(grid_map.required_z_grid)):.3f} m")
    print("")

    root = tk.Tk()

    AstarGUI3DZ(
        root=root,
        grid_map=grid_map,
        preview_path=preview_path,
        output_csv=args.output_simple,
        output_full_csv=args.output,
        output_preview=args.output_preview,
        z_cost_weight=args.z_cost_weight,
        max_expanded=args.max_expanded,
        simplify_z_threshold=args.simplify_z_threshold,
        simplify_max_segment=args.simplify_max_segment,
        add_landing=args.add_landing,
        landing_final_clearance_m=args.landing_final_clearance,
        landing_step_z_m=args.landing_step_z,
    )

    root.mainloop()


if __name__ == "__main__":
    main()
