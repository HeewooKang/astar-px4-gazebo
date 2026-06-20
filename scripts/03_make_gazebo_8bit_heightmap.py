#!/usr/bin/env python3
import argparse
import json
from pathlib import Path

import numpy as np
from PIL import Image


def write_model_files(model_dir):
    model_dir.mkdir(parents=True, exist_ok=True)

    (model_dir / "model.config").write_text("""<?xml version="1.0"?>
<model>
  <name>opentopo_heightmap</name>
  <version>1.0</version>
  <sdf version="1.6">model.sdf</sdf>
  <author>
    <name>terrain_astar_gazebo</name>
  </author>
  <description>DEM heightmap model for Gazebo Classic</description>
</model>
""", encoding="utf-8")

    (model_dir / "model.sdf").write_text("""<?xml version="1.0"?>
<sdf version="1.6">
  <model name="opentopo_heightmap">
    <static>true</static>
    <link name="link"/>
  </model>
</sdf>
""", encoding="utf-8")


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--heightmap",
        default="../output/heightmap.png",
        help="02_make_heightmap_from_dem.py가 만든 원본 heightmap.png"
    )

    parser.add_argument(
        "--meta",
        default="../output/map_meta.json",
        help="map_meta.json 경로"
    )

    parser.add_argument(
        "--output",
        default=str(Path.home() / ".gazebo/models/opentopo_heightmap/materials/textures/heightmap.png"),
        help="Gazebo Classic이 실제로 읽을 8-bit heightmap 저장 경로"
    )

    args = parser.parse_args()

    src = Path(args.heightmap).resolve()
    meta_path = Path(args.meta).resolve()
    dst = Path(args.output).expanduser().resolve()

    if not src.exists():
        raise FileNotFoundError(f"원본 heightmap이 없습니다: {src}")

    if not meta_path.exists():
        raise FileNotFoundError(f"map_meta.json이 없습니다: {meta_path}")

    dst.parent.mkdir(parents=True, exist_ok=True)

    model_dir = Path.home() / ".gazebo/models/opentopo_heightmap"
    write_model_files(model_dir)

    arr = np.array(Image.open(src)).astype(np.float32)

    mn = float(np.nanmin(arr))
    mx = float(np.nanmax(arr))

    if mx <= mn:
        raise RuntimeError(
            "heightmap 값이 전부 같습니다. "
            "DEM 자체가 평평하거나 heightmap 생성이 잘못됐습니다."
        )

    norm = (arr - mn) / (mx - mn)
    img8 = (norm * 255.0).clip(0, 255).astype(np.uint8)

    Image.fromarray(img8, mode="L").save(dst)

    with open(meta_path, "r", encoding="utf-8") as f:
        meta = json.load(f)

    meta["gazebo_model_heightmap_png"] = str(dst)
    meta["gazebo_heightmap_uri"] = "model://opentopo_heightmap/materials/textures/heightmap.png"
    meta["gazebo_heightmap_format"] = "8-bit grayscale PNG for Gazebo Classic"
    meta["gazebo_heightmap_source"] = str(src)

    with open(meta_path, "w", encoding="utf-8") as f:
        json.dump(meta, f, indent=2, ensure_ascii=False)

    print("")
    print("========== Gazebo용 8-bit heightmap 생성 완료 ==========")
    print(f"원본 heightmap      : {src}")
    print(f"Gazebo heightmap   : {dst}")
    print(f"원본 min/max        : {mn:.3f} / {mx:.3f}")
    print(f"변환 min/max        : {int(img8.min())} / {int(img8.max())}")
    print(f"unique count        : {len(np.unique(img8))}")
    print("Gazebo URI          : model://opentopo_heightmap/materials/textures/heightmap.png")
    print(f"map_meta 업데이트   : {meta_path}")
    print("")


if __name__ == "__main__":
    main()
