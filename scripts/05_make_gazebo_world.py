#!/usr/bin/env python3
import argparse
import csv
import json
from pathlib import Path
from xml.sax.saxutils import escape

import numpy as np
from PIL import Image


def read_heightmap_normalized(heightmap_path):
    img = Image.open(heightmap_path)
    arr = np.array(img).astype(np.float32)

    max_val = float(np.nanmax(arr))
    min_val = float(np.nanmin(arr))

    if max_val <= min_val:
        return np.zeros_like(arr, dtype=np.float32)

    norm = (arr - min_val) / (max_val - min_val)
    return np.clip(norm, 0.0, 1.0)


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


def safe_name(text):
    if text is None:
        return ""
    return escape(str(text))


def xy_to_pixel_float(x, y, size_x_m, size_y_m, width_px, height_px):

    col = (x + size_x_m / 2.0) / size_x_m * (width_px - 1)
    row = (size_y_m / 2.0 - y) / size_y_m * (height_px - 1)
    return row, col


def clamp_int(v, lo, hi):
    return max(lo, min(hi, int(v)))


def sample_terrain_stats_under_box(
    height_norm,
    x,
    y,
    width,
    depth,
    size_x_m,
    size_y_m,
    z_size_m,
    padding_m=2.0
):

    height_px, width_px = height_norm.shape

    min_x = x - width / 2.0 - padding_m
    max_x = x + width / 2.0 + padding_m
    min_y = y - depth / 2.0 - padding_m
    max_y = y + depth / 2.0 + padding_m

    row1, col1 = xy_to_pixel_float(min_x, max_y, size_x_m, size_y_m, width_px, height_px)
    row2, col2 = xy_to_pixel_float(max_x, min_y, size_x_m, size_y_m, width_px, height_px)

    r0 = clamp_int(np.floor(min(row1, row2)), 0, height_px - 1)
    r1 = clamp_int(np.ceil(max(row1, row2)), 0, height_px - 1)
    c0 = clamp_int(np.floor(min(col1, col2)), 0, width_px - 1)
    c1 = clamp_int(np.ceil(max(col1, col2)), 0, width_px - 1)

    patch = height_norm[r0:r1 + 1, c0:c1 + 1]

    if patch.size == 0:
        row, col = xy_to_pixel_float(x, y, size_x_m, size_y_m, width_px, height_px)
        r = clamp_int(round(row), 0, height_px - 1)
        c = clamp_int(round(col), 0, width_px - 1)
        patch = height_norm[r:r + 1, c:c + 1]

    z_patch = patch.astype(np.float32) * float(z_size_m)

    return {
        "terrain_min_z": float(np.min(z_patch)),
        "terrain_max_z": float(np.max(z_patch)),
        "terrain_mean_z": float(np.mean(z_patch)),
        "terrain_p90_z": float(np.percentile(z_patch, 90)),
        "sample_rows": int(r1 - r0 + 1),
        "sample_cols": int(c1 - c0 + 1)
    }


def make_box_building_model(
    row,
    terrain_stats,
    index,
    color="0.55 0.55 0.55 1",
    foundation_color="0.35 0.35 0.35 1",
    base_offset=0.20,
    base_mode="max",
    enable_foundation=True
):
    x = safe_float(row.get("x_m"))
    y = safe_float(row.get("y_m"))

    width = max(safe_float(row.get("width_m")), 0.5)
    depth = max(safe_float(row.get("depth_m")), 0.5)
    height = max(safe_float(row.get("height_m"), 10.0), 1.0)

    terrain_min_z = terrain_stats["terrain_min_z"]
    terrain_max_z = terrain_stats["terrain_max_z"]
    terrain_mean_z = terrain_stats["terrain_mean_z"]
    terrain_p90_z = terrain_stats["terrain_p90_z"]

    if base_mode == "mean":
        base_z = terrain_mean_z + base_offset
    elif base_mode == "p90":
        base_z = terrain_p90_z + base_offset
    else:

        base_z = terrain_max_z + base_offset

    main_center_z = base_z + height / 2.0

    csv_id = str(row.get("id", index)).strip()
    if csv_id == "":
        csv_id = str(index)

    osm_type = safe_name(row.get("osm_type", ""))
    osm_id = safe_name(row.get("osm_id", ""))
    building_tag = safe_name(row.get("building_tag", "building"))
    name = safe_name(row.get("name", ""))


    model_name = f"osm_building_{csv_id}"

    foundation_xml = ""

    if enable_foundation:
        foundation_bottom_z = terrain_min_z - 0.05
        foundation_top_z = base_z

        foundation_height = foundation_top_z - foundation_bottom_z

        if foundation_height > 0.10:
            foundation_center_z = foundation_bottom_z + foundation_height / 2.0

            foundation_xml = f"""
        <collision name="foundation_collision">
          <geometry>
            <box>
              <size>{width:.6f} {depth:.6f} {foundation_height:.6f}</size>
            </box>
          </geometry>
          <pose>0 0 {foundation_center_z - main_center_z:.6f} 0 0 0</pose>
        </collision>

        <visual name="foundation_visual">
          <geometry>
            <box>
              <size>{width:.6f} {depth:.6f} {foundation_height:.6f}</size>
            </box>
          </geometry>
          <pose>0 0 {foundation_center_z - main_center_z:.6f} 0 0 0</pose>
          <material>
            <ambient>{foundation_color}</ambient>
            <diffuse>{foundation_color}</diffuse>
          </material>
        </visual>
"""

    return f"""
    <model name="{model_name}">
      <static>true</static>
      <pose>{x:.6f} {y:.6f} {main_center_z:.6f} 0 0 0</pose>

      <link name="link">

        <collision name="main_collision">
          <geometry>
            <box>
              <size>{width:.6f} {depth:.6f} {height:.6f}</size>
            </box>
          </geometry>
        </collision>

        <visual name="main_visual">
          <geometry>
            <box>
              <size>{width:.6f} {depth:.6f} {height:.6f}</size>
            </box>
          </geometry>
          <material>
            <ambient>{color}</ambient>
            <diffuse>{color}</diffuse>
          </material>
        </visual>

{foundation_xml}

      </link>

      <!--
      OSM info:
      csv_id={csv_id}
      osm_type={osm_type}
      osm_id={osm_id}
      tag={building_tag}
      name={name}
      terrain_min_z={terrain_min_z:.3f}
      terrain_max_z={terrain_max_z:.3f}
      terrain_mean_z={terrain_mean_z:.3f}
      terrain_p90_z={terrain_p90_z:.3f}
      base_z={base_z:.3f}
      -->
    </model>
"""


def make_world_xml(
    heightmap_uri,
    size_x_m,
    size_y_m,
    z_size_m,
    buildings_xml,
    world_name="generated_dem_osm_world"
):

    heightmap_pos_z = 0.0

    return f"""<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="{world_name}">

    <physics type="ode">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>250</real_time_update_rate>
      <gravity>0 0 -9.81</gravity>
    </physics>

    <scene>
      <ambient>0.55 0.55 0.55 1</ambient>
      <background>0.70 0.80 1.00 1</background>
      <shadows>true</shadows>
    </scene>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 100 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <model name="dem_heightmap_terrain">
      <static>true</static>
      <pose>0 0 0 0 0 0</pose>

      <link name="terrain_link">

        <collision name="terrain_collision">
          <geometry>
            <heightmap>
              <uri>{heightmap_uri}</uri>
              <size>{size_x_m:.6f} {size_y_m:.6f} {z_size_m:.6f}</size>
              <pos>0 0 {heightmap_pos_z:.6f}</pos>
            </heightmap>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>1.0</mu>
                <mu2>1.0</mu2>
              </ode>
            </friction>
          </surface>
        </collision>

        <visual name="terrain_visual">
          <geometry>
            <heightmap>
              <uri>{heightmap_uri}</uri>
              <size>{size_x_m:.6f} {size_y_m:.6f} {z_size_m:.6f}</size>
              <pos>0 0 {heightmap_pos_z:.6f}</pos>

              <texture>
                <diffuse>file://media/materials/textures/dirt_diffusespecular.png</diffuse>
                <normal>file://media/materials/textures/flat_normal.png</normal>
                <size>20</size>
              </texture>

              <texture>
                <diffuse>file://media/materials/textures/grass_diffusespecular.png</diffuse>
                <normal>file://media/materials/textures/flat_normal.png</normal>
                <size>20</size>
              </texture>

              <texture>
                <diffuse>file://media/materials/textures/fungus_diffusespecular.png</diffuse>
                <normal>file://media/materials/textures/flat_normal.png</normal>
                <size>20</size>
              </texture>

              <blend>
                <min_height>{z_size_m * 0.30:.6f}</min_height>
                <fade_dist>{max(z_size_m * 0.10, 1.0):.6f}</fade_dist>
              </blend>

              <blend>
                <min_height>{z_size_m * 0.65:.6f}</min_height>
                <fade_dist>{max(z_size_m * 0.10, 1.0):.6f}</fade_dist>
              </blend>

            </heightmap>
          </geometry>
        </visual>

      </link>
    </model>

{buildings_xml}

  </world>
</sdf>
"""


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--heightmap", required=True, help="output/heightmap.png")
    parser.add_argument("--meta", required=True, help="output/map_meta.json")
    parser.add_argument("--buildings", required=True, help="output/buildings.csv")
    parser.add_argument("--output", required=True, help="output/generated_world.world")
    parser.add_argument("--max-buildings", type=int, default=500)
    parser.add_argument(
        "--sort-by-area",
        action="store_true",
        help="큰 건물부터 월드에 넣고 싶을 때만 사용. 기본값은 buildings.csv id 순서 유지"
    )
    parser.add_argument("--building-color", default="0.55 0.55 0.55 1")
    parser.add_argument("--foundation-color", default="0.35 0.35 0.35 1")
    parser.add_argument("--base-offset", type=float, default=0.20)
    parser.add_argument(
        "--base-mode",
        choices=["max", "p90", "mean"],
        default="max",
        help="건물 바닥 높이 결정 방식. max=박힘 방지, p90=덜 뜸, mean=평균"
    )
    parser.add_argument("--no-foundation", action="store_true")
    parser.add_argument("--sample-padding", type=float, default=2.0)
    args = parser.parse_args()

    heightmap_path = Path(args.heightmap).resolve()
    meta_path = Path(args.meta).resolve()
    buildings_path = Path(args.buildings).resolve()
    output_path = Path(args.output).resolve()

    if not heightmap_path.exists():
        raise FileNotFoundError(f"heightmap 파일이 없습니다: {heightmap_path}")

    if not meta_path.exists():
        raise FileNotFoundError(f"map_meta.json 파일이 없습니다: {meta_path}")

    if not buildings_path.exists():
        raise FileNotFoundError(f"buildings.csv 파일이 없습니다: {buildings_path}")

    with open(meta_path, "r", encoding="utf-8") as f:
        meta = json.load(f)

    size_x_m = float(meta["gazebo_size_x_m"])
    size_y_m = float(meta["gazebo_size_y_m"])
    z_size_m = float(meta["gazebo_z_size_m"])

    if z_size_m <= 0:
        z_size_m = 1.0

    height_norm = read_heightmap_normalized(heightmap_path)

    buildings = []
    with open(buildings_path, newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            buildings.append(row)

    original_building_count = len(buildings)

    if args.sort_by_area:
        def area_key(row):
            return safe_float(row.get("area_m2"), 0.0)

        buildings = sorted(buildings, key=area_key, reverse=True)

    if args.max_buildings > 0:
        buildings = buildings[:args.max_buildings]

    building_xml_list = []

    debug_rows = []

    for i, row in enumerate(buildings):
        x = safe_float(row.get("x_m"))
        y = safe_float(row.get("y_m"))
        width = max(safe_float(row.get("width_m")), 0.5)
        depth = max(safe_float(row.get("depth_m")), 0.5)

        stats = sample_terrain_stats_under_box(
            height_norm=height_norm,
            x=x,
            y=y,
            width=width,
            depth=depth,
            size_x_m=size_x_m,
            size_y_m=size_y_m,
            z_size_m=z_size_m,
            padding_m=args.sample_padding
        )

        building_xml = make_box_building_model(
            row=row,
            terrain_stats=stats,
            index=i,
            color=args.building_color,
            foundation_color=args.foundation_color,
            base_offset=args.base_offset,
            base_mode=args.base_mode,
            enable_foundation=(not args.no_foundation)
        )

        building_xml_list.append(building_xml)

        csv_id = str(row.get("id", i)).strip()
        if csv_id == "":
            csv_id = str(i)

        debug_rows.append({
            "world_index": i,
            "id": csv_id,
            "model_name": f"osm_building_{csv_id}",
            "osm_type": row.get("osm_type", row.get("VWorld_type", "")),
            "osm_id": row.get("osm_id", row.get("VWorld_id", "")),
            "name": row.get("name", ""),
            "building_tag": row.get("building_tag", ""),
            "height_m": safe_float(row.get("height_m"), 0.0),
            "x_m": x,
            "y_m": y,
            "width_m": width,
            "depth_m": depth,
            "min_x_m": safe_float(row.get("min_x_m"), x - width / 2.0),
            "max_x_m": safe_float(row.get("max_x_m"), x + width / 2.0),
            "min_y_m": safe_float(row.get("min_y_m"), y - depth / 2.0),
            "max_y_m": safe_float(row.get("max_y_m"), y + depth / 2.0),
            "bbox_action": row.get("bbox_action", ""),
            "area_m2": safe_float(row.get("area_m2"), 0.0),
            "terrain_min_z": stats["terrain_min_z"],
            "terrain_max_z": stats["terrain_max_z"],
            "terrain_mean_z": stats["terrain_mean_z"],
            "terrain_p90_z": stats["terrain_p90_z"],
            "sample_rows": stats["sample_rows"],
            "sample_cols": stats["sample_cols"]
        })

    buildings_xml = "\n".join(building_xml_list)

    heightmap_uri = meta.get("gazebo_heightmap_uri", "file://" + str(heightmap_path))

    world_xml = make_world_xml(
        heightmap_uri=heightmap_uri,
        size_x_m=size_x_m,
        size_y_m=size_y_m,
        z_size_m=z_size_m,
        buildings_xml=buildings_xml
    )

    output_path.parent.mkdir(parents=True, exist_ok=True)

    with open(output_path, "w", encoding="utf-8") as f:
        f.write(world_xml)

    debug_path = output_path.parent / "building_terrain_debug.csv"
    with open(debug_path, "w", newline="", encoding="utf-8") as f:
        fieldnames = [
            "world_index",
            "id",
            "model_name",
            "osm_type",
            "osm_id",
            "name",
            "building_tag",
            "height_m",
            "x_m",
            "y_m",
            "width_m",
            "depth_m",
            "min_x_m",
            "max_x_m",
            "min_y_m",
            "max_y_m",
            "bbox_action",
            "area_m2",
            "terrain_min_z",
            "terrain_max_z",
            "terrain_mean_z",
            "terrain_p90_z",
            "sample_rows",
            "sample_cols"
        ]
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for r in debug_rows:
            writer.writerow(r)

    print("")
    print("========== Gazebo World 생성 완료 ==========")
    print(f"heightmap             : {heightmap_path}")
    print(f"map_meta              : {meta_path}")
    print(f"buildings.csv         : {buildings_path}")
    print("")
    print(f"Gazebo size X         : {size_x_m:.3f} m")
    print(f"Gazebo size Y         : {size_y_m:.3f} m")
    print(f"Gazebo size Z         : {z_size_m:.3f} m")
    print("")
    print(f"전체 건물 개수         : {original_building_count}")
    print(f"월드에 넣은 건물 수    : {len(buildings)}")
    print(f"ID 유지 방식           : buildings.csv의 id를 model name/debug id에 그대로 사용")
    print(f"면적순 정렬            : {args.sort_by_area}")
    print(f"건물 바닥 방식         : {args.base_mode}")
    print(f"건물 바닥 offset       : {args.base_offset:.3f} m")
    print(f"기초 박스 사용         : {not args.no_foundation}")
    print("")
    print(f"world 저장             : {output_path}")
    print(f"디버그 저장            : {debug_path}")
    print("")
    print("테스트 실행:")
    print(f"gazebo --verbose {output_path}")
    print("")


if __name__ == "__main__":
    main()
