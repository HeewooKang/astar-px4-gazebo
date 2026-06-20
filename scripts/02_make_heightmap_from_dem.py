#!/usr/bin/env python3
import argparse
import json
import math
import shutil
from pathlib import Path

import numpy as np
from PIL import Image

import rasterio
from rasterio.enums import Resampling
from rasterio.transform import from_bounds
from rasterio.warp import reproject


EARTH_R = 6378137.0


def meters_from_latlon_bbox(min_lat, max_lat, min_lon, max_lon):
    center_lat = (min_lat + max_lat) / 2.0

    size_y_m = abs(math.radians(max_lat - min_lat) * EARTH_R)
    size_x_m = abs(
        math.radians(max_lon - min_lon)
        * EARTH_R
        * math.cos(math.radians(center_lat))
    )

    return size_x_m, size_y_m


def next_valid_heightmap_size(requested):
    valid_sizes = [129, 257, 513, 1025, 2049]
    for s in valid_sizes:
        if requested <= s:
            return s
    return 2049

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--dem", required=True, help="Input DEM GeoTIFF path")
    parser.add_argument("--bbox", required=True, help="bbox.json path")
    parser.add_argument("--output-dir", required=True, help="Output directory")
    parser.add_argument("--size", type=int, default=513, help="Heightmap size. Use 513 or 1025")
    parser.add_argument("--vertical-exaggeration", type=float, default=10.0)
    parser.add_argument("--min-z-size", type=float, default=80.0)
    parser.add_argument("--force-z-size", type=float, default=0.0)
    args = parser.parse_args()

    dem_path = Path(args.dem).resolve()
    bbox_path = Path(args.bbox).resolve()
    output_dir = Path(args.output_dir).resolve()

    if not dem_path.exists():
        raise FileNotFoundError(f"DEM 파일이 없습니다: {dem_path}")

    if not bbox_path.exists():
        raise FileNotFoundError(f"bbox.json 파일이 없습니다: {bbox_path}")

    output_dir.mkdir(parents=True, exist_ok=True)

    with open(bbox_path, "r", encoding="utf-8") as f:
        bbox = json.load(f)

    min_lat = float(bbox["min_lat"])
    max_lat = float(bbox["max_lat"])
    min_lon = float(bbox["min_lon"])
    max_lon = float(bbox["max_lon"])
    center_lat = float(bbox["center_lat"])
    center_lon = float(bbox["center_lon"])

    target_size = next_valid_heightmap_size(args.size)

    dst_nodata = -9999.0

    with rasterio.open(dem_path) as src:
        if src.crs is None:
            raise RuntimeError(
                "DEM GeoTIFF에 CRS 좌표계 정보가 없습니다. "
                "GeoTIFF가 아니면 위도/경도 정렬 heightmap을 만들 수 없습니다."
            )

        dem_crs = str(src.crs)
        dem_width = src.width
        dem_height = src.height
        dem_nodata = src.nodata

        dst_transform = from_bounds(
            min_lon,
            min_lat,
            max_lon,
            max_lat,
            target_size,
            target_size
        )

        arr = np.full((target_size, target_size), dst_nodata, dtype=np.float32)

        reproject(
            source=rasterio.band(src, 1),
            destination=arr,
            src_transform=src.transform,
            src_crs=src.crs,
            src_nodata=src.nodata,
            dst_transform=dst_transform,
            dst_crs="EPSG:4326",
            dst_nodata=dst_nodata,
            resampling=Resampling.bilinear
        )

    arr = arr.astype(np.float32)
    arr[arr == dst_nodata] = np.nan
    arr[~np.isfinite(arr)] = np.nan

    if np.all(np.isnan(arr)):
        raise RuntimeError(
            "bbox 영역에서 DEM 값을 하나도 읽지 못했습니다.\n"
            "확인 필요:\n"
            "1. bbox.json이 현재 DEM에서 만든 파일인지\n"
            "2. DEM 파일과 bbox 영역이 같은 지역인지\n"
            "3. DEM에 NoData만 있는지"
        )

    valid_min = float(np.nanmin(arr))
    valid_max = float(np.nanmax(arr))
    valid_mean = float(np.nanmean(arr))

    arr = np.where(np.isfinite(arr), arr, valid_min).astype(np.float32)

    elev_min = float(np.min(arr))
    elev_max = float(np.max(arr))
    elev_range_real = elev_max - elev_min

    if elev_range_real <= 0.0001:
        print("[WARN] DEM 고도 차이가 거의 없습니다. 실제 지형 자체가 거의 평지일 수 있습니다.")
        elev_range_for_norm = 1.0
    else:
        elev_range_for_norm = elev_range_real

    if args.force_z_size > 0:
        gazebo_z_size_m = float(args.force_z_size)
    else:
        gazebo_z_size_m = max(
            elev_range_real * float(args.vertical_exaggeration),
            float(args.min_z_size)
        )

    norm = (arr - elev_min) / (elev_range_for_norm + 1e-9)
    norm = np.clip(norm, 0.0, 1.0)

    img8 = (norm * 255.0).astype(np.uint8)

    heightmap_path = output_dir / "heightmap.png"
    preview_path = output_dir / "heightmap_preview.png"
    debug_16_path = output_dir / "heightmap_16bit_debug.png"

    Image.fromarray(img8, mode="L").save(heightmap_path)
    Image.fromarray(img8, mode="L").save(preview_path)

    img16 = (norm * 65535.0).astype(np.uint16)
    Image.fromarray(img16, mode="I;16").save(debug_16_path)

    size_x_m, size_y_m = meters_from_latlon_bbox(
        min_lat=min_lat,
        max_lat=max_lat,
        min_lon=min_lon,
        max_lon=max_lon
    )

    map_meta = {
        "input_dem": str(dem_path),
        "dem_crs": dem_crs,
        "dem_width_px": dem_width,
        "dem_height_px": dem_height,
        "dem_nodata": dem_nodata,

        "heightmap_png": str(heightmap_path),
        "heightmap_preview_png": str(preview_path),
        "heightmap_16bit_debug_png": str(debug_16_path),
        "heightmap_size_px": target_size,

        "min_lat": min_lat,
        "max_lat": max_lat,
        "min_lon": min_lon,
        "max_lon": max_lon,
        "center_lat": center_lat,
        "center_lon": center_lon,

        "gazebo_origin_rule": "center of bbox is Gazebo x=0, y=0",
        "gazebo_x_positive": "east",
        "gazebo_y_positive": "north",
        "gazebo_z_positive": "up",

        "gazebo_size_x_m": size_x_m,
        "gazebo_size_y_m": size_y_m,
        "gazebo_z_size_m": gazebo_z_size_m,

        "elevation_min_m": elev_min,
        "elevation_max_m": elev_max,
        "elevation_range_m": elev_range_real,
        "elevation_mean_m": valid_mean,
        "vertical_exaggeration": args.vertical_exaggeration,
        "min_z_size_m": args.min_z_size,
        "force_z_size_m": args.force_z_size,

        "note": "Heightmap is reprojected to EPSG:4326 bbox grid and copied into ~/.gazebo/models/opentopo_heightmap for stable model:// loading."
    }

    meta_path = output_dir / "map_meta.json"

    with open(meta_path, "w", encoding="utf-8") as f:
        json.dump(map_meta, f, indent=2, ensure_ascii=False)

    print("")
    print("========== Heightmap 생성 완료 ==========")
    print(f"DEM 입력                  : {dem_path}")
    print(f"DEM CRS                   : {dem_crs}")
    print(f"원본 DEM 크기             : {dem_width} x {dem_height}")
    print(f"heightmap 크기            : {target_size} x {target_size}")
    print("")
    print(f"bbox lat                  : {min_lat:.8f} ~ {max_lat:.8f}")
    print(f"bbox lon                  : {min_lon:.8f} ~ {max_lon:.8f}")
    print("")
    print(f"Gazebo size X             : {size_x_m:.3f} m")
    print(f"Gazebo size Y             : {size_y_m:.3f} m")
    print(f"Gazebo size Z             : {gazebo_z_size_m:.3f} m")
    print("")
    print(f"DEM 최저 고도             : {elev_min:.3f} m")
    print(f"DEM 최고 고도             : {elev_max:.3f} m")
    print(f"DEM 실제 고도 차이        : {elev_range_real:.3f} m")
    print("")
    print(f"heightmap 저장            : {heightmap_path}")
    print(f"map_meta 저장             : {meta_path}")
    print("")


if __name__ == "__main__":
    main()
