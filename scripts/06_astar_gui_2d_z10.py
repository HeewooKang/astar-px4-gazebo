#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import csv
import heapq
import json
import math
from collections import deque
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


class GridMap2DFixedZ:
    def __init__(
        self,
        map_meta,
        heightmap_path,
        buildings,
        resolution_m=5.0,
        safety_margin_m=5.0,
        fixed_z_m=10.0
    ):
        self.meta = map_meta
        self.heightmap_path = Path(heightmap_path)
        self.buildings = buildings

        self.size_x_m = float(map_meta["gazebo_size_x_m"])
        self.size_y_m = float(map_meta["gazebo_size_y_m"])
        self.z_size_m = float(map_meta["gazebo_z_size_m"])

        self.resolution_m = float(resolution_m)
        self.safety_margin_m = float(safety_margin_m)
        self.fixed_z_m = float(fixed_z_m)

        self.cols = max(2, int(math.ceil(self.size_x_m / self.resolution_m)))
        self.rows = max(2, int(math.ceil(self.size_y_m / self.resolution_m)))

        if self.cols > 1500 or self.rows > 1500:
            raise RuntimeError(
                f"Grid가 너무 큽니다: {self.cols} x {self.rows}\n"
                f"--resolution 값을 더 크게 주세요. 예: --resolution 10"
            )

        self.height_norm = read_heightmap_normalized(self.heightmap_path)
        self.occupancy = np.zeros((self.rows, self.cols), dtype=np.uint8)

        self.mark_building_obstacles()

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

    def world_to_height_pixel(self, x, y):
        h, w = self.height_norm.shape

        px = (x + self.size_x_m / 2.0) / self.size_x_m * (w - 1)
        py = (self.size_y_m / 2.0 - y) / self.size_y_m * (h - 1)

        px = int(round(max(0, min(w - 1, px))))
        py = int(round(max(0, min(h - 1, py))))

        return py, px

    def terrain_z(self, x, y):
        py, px = self.world_to_height_pixel(x, y)
        return float(self.height_norm[py, px]) * self.z_size_m

    def mark_building_obstacles(self):

        margin = self.safety_margin_m

        for b in self.buildings:
            min_x = b["min_x"] - margin
            max_x = b["max_x"] + margin
            min_y = b["min_y"] - margin
            max_y = b["max_y"] + margin

            r1, c1 = self.world_to_grid(min_x, max_y)
            r2, c2 = self.world_to_grid(max_x, min_y)

            r0 = max(0, min(r1, r2))
            r3 = min(self.rows - 1, max(r1, r2))
            c0 = max(0, min(c1, c2))
            c3 = min(self.cols - 1, max(c1, c2))

            self.occupancy[r0:r3 + 1, c0:c3 + 1] = 1

    def is_free(self, row, col):
        if row < 0 or row >= self.rows or col < 0 or col >= self.cols:
            return False
        return self.occupancy[row, col] == 0

    def nearest_free_cell(self, row, col, max_radius=80):
        if self.is_free(row, col):
            return row, col

        visited = set()
        q = deque()
        q.append((row, col, 0))
        visited.add((row, col))

        directions = [
            (-1, 0), (1, 0), (0, -1), (0, 1),
            (-1, -1), (-1, 1), (1, -1), (1, 1)
        ]

        while q:
            r, c, d = q.popleft()

            if d > max_radius:
                break

            for dr, dc in directions:
                nr = r + dr
                nc = c + dc

                if (nr, nc) in visited:
                    continue

                if nr < 0 or nr >= self.rows or nc < 0 or nc >= self.cols:
                    continue

                if self.is_free(nr, nc):
                    return nr, nc

                visited.add((nr, nc))
                q.append((nr, nc, d + 1))

        return None


def astar_2d(grid_map, start, goal):
    rows = grid_map.rows
    cols = grid_map.cols
    occ = grid_map.occupancy

    sr, sc = start
    gr, gc = goal

    def heuristic(r, c):
        dx = abs(c - gc)
        dy = abs(r - gr)
        return (dx + dy) + (math.sqrt(2) - 2) * min(dx, dy)

    neighbors = [
        (-1, 0, 1.0),
        (1, 0, 1.0),
        (0, -1, 1.0),
        (0, 1, 1.0),
        (-1, -1, math.sqrt(2)),
        (-1, 1, math.sqrt(2)),
        (1, -1, math.sqrt(2)),
        (1, 1, math.sqrt(2)),
    ]

    open_heap = []
    heapq.heappush(open_heap, (heuristic(sr, sc), 0.0, sr, sc))

    came_from = {}
    g_score = {(sr, sc): 0.0}
    visited = set()

    while open_heap:
        _, current_g, r, c = heapq.heappop(open_heap)

        if (r, c) in visited:
            continue

        visited.add((r, c))

        if (r, c) == (gr, gc):
            path = [(r, c)]
            while (r, c) in came_from:
                r, c = came_from[(r, c)]
                path.append((r, c))
            path.reverse()
            return path

        for dr, dc, cost in neighbors:
            nr = r + dr
            nc = c + dc

            if nr < 0 or nr >= rows or nc < 0 or nc >= cols:
                continue

            if occ[nr, nc] != 0:
                continue

            if dr != 0 and dc != 0:
                if occ[r, nc] != 0 or occ[nr, c] != 0:
                    continue

            tentative_g = current_g + cost

            if tentative_g < g_score.get((nr, nc), float("inf")):
                came_from[(nr, nc)] = (r, c)
                g_score[(nr, nc)] = tentative_g
                f = tentative_g + heuristic(nr, nc)
                heapq.heappush(open_heap, (f, tentative_g, nr, nc))

    return None


def simplify_path_by_direction(path):
    if path is None or len(path) <= 2:
        return path

    result = [path[0]]

    prev_dr = None
    prev_dc = None

    for i in range(1, len(path)):
        r0, c0 = path[i - 1]
        r1, c1 = path[i]

        dr = int(math.copysign(1, r1 - r0)) if r1 != r0 else 0
        dc = int(math.copysign(1, c1 - c0)) if c1 != c0 else 0

        if prev_dr is None:
            prev_dr = dr
            prev_dc = dc
            continue

        if dr != prev_dr or dc != prev_dc:
            result.append(path[i - 1])
            prev_dr = dr
            prev_dc = dc

    result.append(path[-1])
    return result


def build_path_rows(
    grid_map,
    path,
    add_landing=True,
    landing_final_z_m=0.0,
    landing_step_z_m=1.0
):

    rows = []

    for i, (row, col) in enumerate(path):
        x, y = grid_map.grid_to_world(row, col)
        terrain_z = grid_map.terrain_z(x, y)

        rows.append({
            "index": i,
            "grid_row": row,
            "grid_col": col,
            "gazebo_x_m": f"{x:.6f}",
            "gazebo_y_m": f"{y:.6f}",
            "gazebo_z_m": f"{grid_map.fixed_z_m:.6f}",
            "terrain_z_m": f"{terrain_z:.6f}",
            "fixed_z_m": f"{grid_map.fixed_z_m:.6f}",
            "planner_mode": "2d_fixed_z_buildings_as_obstacles",
            "altitude_reason": "fixed_z"
        })

    if not add_landing or not rows:
        return rows

    step = abs(float(landing_step_z_m))
    if step <= 0.0:
        step = 1.0

    start_z = float(rows[-1]["gazebo_z_m"])
    final_z = float(landing_final_z_m)

    if start_z <= final_z:
        return rows

    last = rows[-1].copy()
    descent_count = int(math.ceil((start_z - final_z) / step))

    for k in range(1, descent_count + 1):
        z = max(final_z, start_z - step * k)

        landing_row = last.copy()
        landing_row["index"] = len(rows)
        landing_row["gazebo_z_m"] = f"{z:.6f}"
        landing_row["planner_mode"] = "2d_fixed_z_buildings_as_obstacles_with_landing"
        landing_row["altitude_reason"] = "landing_descent"

        rows.append(landing_row)

    return rows


def write_path_csv(
    grid_map,
    path,
    output_csv,
    add_landing=True,
    landing_final_z_m=0.0,
    landing_step_z_m=1.0
):
    output_csv = Path(output_csv)
    output_csv.parent.mkdir(parents=True, exist_ok=True)

    fieldnames = [
        "index",
        "grid_row",
        "grid_col",
        "gazebo_x_m",
        "gazebo_y_m",
        "gazebo_z_m",
        "terrain_z_m",
        "fixed_z_m",
        "planner_mode",
        "altitude_reason"
    ]

    rows = build_path_rows(
        grid_map=grid_map,
        path=path,
        add_landing=add_landing,
        landing_final_z_m=landing_final_z_m,
        landing_step_z_m=landing_step_z_m
    )

    with open(output_csv, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)



class AstarGUI2DFixedZ:
    def __init__(
        self,
        root,
        grid_map,
        preview_path,
        output_csv,
        output_full_csv,
        output_preview,
        add_landing=True,
        landing_final_z_m=0.0,
        landing_step_z_m=1.0
    ):
        self.root = root
        self.grid_map = grid_map
        self.preview_path = Path(preview_path)
        self.output_csv = Path(output_csv)
        self.output_full_csv = Path(output_full_csv)

        if output_preview is None:
            self.output_preview = self.output_full_csv.parent / "path_preview_2d_z10.png"
        else:
            self.output_preview = Path(output_preview)

        self.add_landing = bool(add_landing)
        self.landing_final_z_m = float(landing_final_z_m)
        self.landing_step_z_m = float(landing_step_z_m)

        self.start_cell = None
        self.goal_cell = None
        self.path = None
        self.path_simplified = None

        self.max_display_w = 1100
        self.max_display_h = 850

        self.base_img = self.make_base_image()
        self.display_img = self.base_img.copy()

        self.img_w, self.img_h = self.display_img.size

        self.root.title("A* 2D Fixed Z=10m - Buildings are Obstacles")

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
                fill=(255, 0, 0, 90),
                outline=(180, 0, 0, 180),
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
            f"Mode: 2D A*, Buildings = Obstacles, Fixed Z = {self.grid_map.fixed_z_m:.2f} m, "
            f"Landing: {'ON' if self.add_landing else 'OFF'} -> z={self.landing_final_z_m:.2f} m, "
            f"Grid: {self.grid_map.cols} x {self.grid_map.rows}, "
            f"Resolution: {self.grid_map.resolution_m:.2f} m, "
            f"Safety margin: {self.grid_map.safety_margin_m:.2f} m"
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

        if self.path is not None:
            pts = []
            for row, col in self.path:
                x, y = self.grid_map.grid_to_world(row, col)
                pts.append(self.world_to_image(x, y))

            if len(pts) >= 2:
                draw.line(pts, fill=(255, 255, 0, 230), width=5)
                draw.line(pts, fill=(255, 0, 0, 255), width=2)

        if self.path_simplified is not None:
            for row, col in self.path_simplified:
                x, y = self.grid_map.grid_to_world(row, col)
                px, py = self.world_to_image(x, y)
                r = 3
                draw.ellipse([px - r, py - r, px + r, py + r], fill=(255, 255, 255, 255), outline=(0, 0, 0, 255))

        self.display_img = img.convert("RGB")
        self.tk_img = ImageTk.PhotoImage(self.display_img)
        self.canvas.itemconfig(self.canvas_img_id, image=self.tk_img)

    def on_left_click(self, event):
        x, y = self.image_to_world(event.x, event.y)
        row, col = self.grid_map.world_to_grid(x, y)

        free = self.grid_map.nearest_free_cell(row, col)

        if free is None:
            messagebox.showerror("Error", "클릭한 지점 주변에서 이동 가능한 free cell을 찾지 못했습니다.")
            return

        if free != (row, col):
            print("[WARN] 클릭 지점이 건물 장애물입니다. 가장 가까운 free cell로 이동합니다:", free)

        if self.start_cell is None:
            self.start_cell = free
            self.goal_cell = None
            self.path = None
            self.path_simplified = None
            print("START:", self.start_cell, self.grid_map.grid_to_world(*self.start_cell))

        elif self.goal_cell is None:
            self.goal_cell = free
            print("GOAL :", self.goal_cell, self.grid_map.grid_to_world(*self.goal_cell))
            self.run_astar()

        else:
            self.start_cell = free
            self.goal_cell = None
            self.path = None
            self.path_simplified = None
            print("새 START:", self.start_cell, self.grid_map.grid_to_world(*self.start_cell))

        self.refresh_image()

    def run_astar(self):
        if self.start_cell is None or self.goal_cell is None:
            return

        print("")
        print("========== 2D Fixed-Z A* 경로 계산 시작 ==========")
        print("start:", self.start_cell)
        print("goal :", self.goal_cell)

        path = astar_2d(self.grid_map, self.start_cell, self.goal_cell)

        if path is None:
            self.path = None
            self.path_simplified = None
            messagebox.showerror("A* 실패", "경로를 찾지 못했습니다. safety-margin이나 resolution을 조정하세요.")
            print("[FAIL] 경로 없음")
            return

        simplified = simplify_path_by_direction(path)

        self.path = path
        self.path_simplified = simplified

        write_path_csv(
            self.grid_map,
            path,
            self.output_full_csv,
            add_landing=self.add_landing,
            landing_final_z_m=self.landing_final_z_m,
            landing_step_z_m=self.landing_step_z_m
        )
        write_path_csv(
            self.grid_map,
            simplified,
            self.output_csv,
            add_landing=self.add_landing,
            landing_final_z_m=self.landing_final_z_m,
            landing_step_z_m=self.landing_step_z_m
        )

        print("A* full path cell 개수 :", len(path))
        print("간소화 waypoint 개수  :", len(simplified))
        print("고정 z값              :", self.grid_map.fixed_z_m)
        print("착륙 waypoint 추가    :", self.add_landing)
        if self.add_landing:
            print("착륙 최종 z           :", self.landing_final_z_m)
            print("착륙 z step           :", self.landing_step_z_m)
        print("비행용 full path 저장:", self.output_full_csv)
        print("간소화 path 저장     :", self.output_csv)
        print("")

        self.refresh_image()
        self.output_preview.parent.mkdir(parents=True, exist_ok=True)
        self.display_img.save(self.output_preview)

        messagebox.showinfo(
            "2D Fixed-Z A* 완료",
            f"경로 생성 완료!\n\n"
            f"전체 cell 개수: {len(path)}\n"
            f"waypoint 개수: {len(simplified)}\n"
            f"모든 waypoint z: {self.grid_map.fixed_z_m:.2f} m\n"
            f"착륙 waypoint: {'ON' if self.add_landing else 'OFF'}"
            f"{f' / final z={self.landing_final_z_m:.2f} m' if self.add_landing else ''}\n\n"
            f"비행용 full path:\n{self.output_full_csv}\n\n"
            f"간소화 path:\n{self.output_csv}"
        )

    def save_current_path(self):
        if self.path is None or self.path_simplified is None:
            messagebox.showwarning("No path", "아직 생성된 경로가 없습니다.")
            return

        write_path_csv(
            self.grid_map,
            self.path,
            self.output_full_csv,
            add_landing=self.add_landing,
            landing_final_z_m=self.landing_final_z_m,
            landing_step_z_m=self.landing_step_z_m,
        )
        write_path_csv(
            self.grid_map,
            self.path_simplified,
            self.output_csv,
            add_landing=self.add_landing,
            landing_final_z_m=self.landing_final_z_m,
            landing_step_z_m=self.landing_step_z_m,
        )

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
        self.path = None
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
        default="../output/path_full_2d_z10.csv",
        help="드론 비행용 전체 경로 CSV"
    )
    parser.add_argument(
        "--output-simple",
        default="../output/path_2d_z10.csv",
        help="확인/시각화용 간소화 경로 CSV"
    )
    parser.add_argument(
        "--output-preview",
        default=None,
        help="경로 미리보기 이미지 저장 경로"
    )
    parser.add_argument("--resolution", type=float, default=5.0, help="A* grid resolution in meters")
    parser.add_argument("--safety-margin", type=float, default=5.0, help="building obstacle safety margin in meters")
    parser.add_argument("--fixed-z", type=float, default=10.0, help="모든 waypoint에 넣을 고정 gazebo_z_m")
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
        "--landing-final-z",
        type=float,
        default=0.0,
        help="2D z=10 모드의 착륙 최종 z. MAVROS local z 기준이면 0이 지면 착륙 목표"
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
    print("========== 2D Fixed-Z A* GUI 로딩 ==========")
    print(f"meta          : {meta_path}")
    print(f"heightmap     : {heightmap_path}")
    print(f"preview       : {preview_path}")
    print(f"buildings     : {buildings_path}")
    print(f"building count: {len(buildings)}")
    print(f"resolution    : {args.resolution} m")
    print(f"safety margin : {args.safety_margin} m")
    print(f"fixed z       : {args.fixed_z} m")
    print(f"add landing   : {args.add_landing}")
    print(f"landing final z: {args.landing_final_z} m")
    print(f"landing step z : {args.landing_step_z} m")
    print("")

    grid_map = GridMap2DFixedZ(
        map_meta=meta,
        heightmap_path=heightmap_path,
        buildings=buildings,
        resolution_m=args.resolution,
        safety_margin_m=args.safety_margin,
        fixed_z_m=args.fixed_z
    )

    print("Grid size:", grid_map.cols, "x", grid_map.rows)
    print("Obstacle cells:", int(np.sum(grid_map.occupancy)))
    print("")

    root = tk.Tk()

    AstarGUI2DFixedZ(
        root=root,
        grid_map=grid_map,
        preview_path=preview_path,
        output_csv=args.output_simple,
        output_full_csv=args.output,
        output_preview=args.output_preview,
        add_landing=args.add_landing,
        landing_final_z_m=args.landing_final_z,
        landing_step_z_m=args.landing_step_z
    )

    root.mainloop()


if __name__ == "__main__":
    main()
