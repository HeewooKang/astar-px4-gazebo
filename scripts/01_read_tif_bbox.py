#!/usr/bin/env python3
import argparse
import json
from pathlib import Path

import rasterio
from rasterio.warp import transform_bounds


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", required=True, help="Input GeoTIFF path")
    parser.add_argument("--output", required=True, help="Output bbox json path")
    args = parser.parse_args()

    input_path = Path(args.input)
    output_path = Path(args.output)

    if not input_path.exists():
        raise FileNotFoundError(f"입력 tif 파일이 없습니다: {input_path}")

    with rasterio.open(input_path) as src:
        if src.crs is None:
            raise RuntimeError(
                "이 tif 파일에는 CRS 좌표계 정보가 없습니다. "
                "GeoTIFF가 아니면 DEM/OSM 좌표 정렬을 할 수 없습니다."
            )

        bounds = src.bounds
        crs = src.crs

        min_lon, min_lat, max_lon, max_lat = transform_bounds(
            crs,
            "EPSG:4326",
            bounds.left,
            bounds.bottom,
            bounds.right,
            bounds.top,
            densify_pts=21
        )

        center_lat = (min_lat + max_lat) / 2.0
        center_lon = (min_lon + max_lon) / 2.0

        result = {
            "input_file": str(input_path),
            "source_crs": str(crs),
            "width_px": src.width,
            "height_px": src.height,
            "min_lat": min_lat,
            "max_lat": max_lat,
            "min_lon": min_lon,
            "max_lon": max_lon,
            "center_lat": center_lat,
            "center_lon": center_lon
        }

    output_path.parent.mkdir(parents=True, exist_ok=True)

    with open(output_path, "w", encoding="utf-8") as f:
        json.dump(result, f, indent=2, ensure_ascii=False)

    print("")
    print("========== GeoTIFF 좌표 범위 ==========")
    print(f"input      : {input_path}")
    print(f"source CRS : {result['source_crs']}")
    print(f"size px    : {result['width_px']} x {result['height_px']}")
    print("")
    print(f"min_lat    : {min_lat:.8f}")
    print(f"max_lat    : {max_lat:.8f}")
    print(f"min_lon    : {min_lon:.8f}")
    print(f"max_lon    : {max_lon:.8f}")
    print("")
    print(f"center_lat : {center_lat:.8f}")
    print(f"center_lon : {center_lon:.8f}")
    print("")
    print(f"저장 완료  : {output_path}")


if __name__ == "__main__":
    main()
