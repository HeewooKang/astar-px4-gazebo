#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import csv
import json
import math
import os
import re
import sys
import time
import urllib.parse
import urllib.request
from collections import Counter
from pathlib import Path

EARTH_R = 6378137.0


def safe_float(value, default=None):
    try:
        if value is None:
            return default
        s = str(value).strip()
        if s == "":
            return default
        s = s.replace(",", "")
        return float(s)
    except Exception:
        return default


def parse_float_first(value, default=None):
    if value is None:
        return default
    s = str(value).strip().replace(",", "")
    if not s:
        return default
    m = re.search(r"[-+]?\d*\.?\d+", s)
    if not m:
        return default
    try:
        return float(m.group(0))
    except Exception:
        return default


def latlon_to_xy(lat, lon, center_lat, center_lon):
    lat = float(lat)
    lon = float(lon)
    x = math.radians(lon - center_lon) * EARTH_R * math.cos(math.radians(center_lat))
    y = math.radians(lat - center_lat) * EARTH_R
    return x, y


def polygon_area_xy(points):
    if len(points) < 3:
        return 0.0
    area = 0.0
    n = len(points)
    for i in range(n):
        x1, y1 = points[i]
        x2, y2 = points[(i + 1) % n]
        area += x1 * y2 - x2 * y1
    return abs(area) * 0.5


def ensure_closed(points):
    if not points:
        return points
    if points[0] != points[-1]:
        return points + [points[0]]
    return points


def write_csv_rows(path, rows, fieldnames, encoding="utf-8"):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", newline="", encoding=encoding) as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def read_json(path):
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def save_json(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, ensure_ascii=False)


def clip_polygon_rect(points, min_x, max_x, min_y, max_y):

    if len(points) < 3:
        return []

    pts = points[:]
    if len(pts) >= 2 and pts[0] == pts[-1]:
        pts = pts[:-1]

    def clip_edge(poly, inside, intersect):
        if not poly:
            return []
        out = []
        s = poly[-1]
        s_inside = inside(s)
        for e in poly:
            e_inside = inside(e)
            if e_inside:
                if not s_inside:
                    out.append(intersect(s, e))
                out.append(e)
            elif s_inside:
                out.append(intersect(s, e))
            s = e
            s_inside = e_inside
        return out

    def inter_x_const(s, e, x_const):
        x1, y1 = s
        x2, y2 = e
        if abs(x2 - x1) < 1e-12:
            return (x_const, y1)
        t = (x_const - x1) / (x2 - x1)
        return (x_const, y1 + t * (y2 - y1))

    def inter_y_const(s, e, y_const):
        x1, y1 = s
        x2, y2 = e
        if abs(y2 - y1) < 1e-12:
            return (x1, y_const)
        t = (y_const - y1) / (y2 - y1)
        return (x1 + t * (x2 - x1), y_const)

    poly = pts
    poly = clip_edge(poly, lambda p: p[0] >= min_x, lambda s, e: inter_x_const(s, e, min_x))
    poly = clip_edge(poly, lambda p: p[0] <= max_x, lambda s, e: inter_x_const(s, e, max_x))
    poly = clip_edge(poly, lambda p: p[1] >= min_y, lambda s, e: inter_y_const(s, e, min_y))
    poly = clip_edge(poly, lambda p: p[1] <= max_y, lambda s, e: inter_y_const(s, e, max_y))

    cleaned = []
    for p in poly:
        if not cleaned:
            cleaned.append(p)
        else:
            if math.hypot(p[0] - cleaned[-1][0], p[1] - cleaned[-1][1]) > 1e-6:
                cleaned.append(p)

    if len(cleaned) >= 2 and math.hypot(cleaned[0][0] - cleaned[-1][0], cleaned[0][1] - cleaned[-1][1]) < 1e-6:
        cleaned = cleaned[:-1]

    if len(cleaned) < 3:
        return []
    return ensure_closed(cleaned)


def build_url(base_url, params):
    return base_url + "?" + urllib.parse.urlencode(params, doseq=False)


def http_get_json(url, timeout=40, sleep_sec=0.0):
    if sleep_sec > 0:
        time.sleep(sleep_sec)
    req = urllib.request.Request(
        url,
        headers={
            "User-Agent": "terrain_astar_gazebo_vworld/1.0",
            "Accept": "application/json,text/javascript,text/plain,*/*",
        },
        method="GET",
    )
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        raw = resp.read().decode("utf-8", errors="replace")

    text = raw.strip()
    if text.startswith("parseResponse(") and text.endswith(")"):
        text = text[len("parseResponse("):-1]
    else:
        m = re.match(r"^[A-Za-z_][A-Za-z0-9_]*\((.*)\)\s*;?\s*$", text, re.DOTALL)
        if m:
            text = m.group(1)

    try:
        return json.loads(text), raw
    except json.JSONDecodeError:
        raise RuntimeError(
            "API 응답이 JSON이 아닙니다. 키/도메인/typename/응답 포맷을 확인하세요.\n"
            f"URL: {url}\n"
            f"응답 앞부분:\n{raw[:1000]}"
        )


def call_vworld_wfs(bbox, api_key, domain, typename, maxfeatures, output, timeout):
    min_lat = float(bbox["min_lat"])
    max_lat = float(bbox["max_lat"])
    min_lon = float(bbox["min_lon"])
    max_lon = float(bbox["max_lon"])

    bbox_text = f"{min_lat},{min_lon},{max_lat},{max_lon}"

    params = {
        "SERVICE": "WFS",
        "REQUEST": "GetFeature",
        "VERSION": "1.1.0",
        "TYPENAME": typename,
        "BBOX": bbox_text,
        "SRSNAME": "EPSG:4326",
        "OUTPUT": output,
        "MAXFEATURES": str(maxfeatures),
        "EXCEPTIONS": "text/xml",
        "KEY": api_key,
    }
    if domain:
        params["DOMAIN"] = domain

    url = build_url("https://api.vworld.kr/req/wfs", params)

    print("\n========== VWorld WFS 요청 ==========")
    print(f"typename : {typename}")
    print(f"bbox     : {bbox_text}  # EPSG:4326 ymin,xmin,ymax,xmax")
    print(f"output   : {output}")
    print(f"domain   : {domain if domain else '(없음)'}")
    print(f"url      : {url.replace(api_key, '[VWORLD_API_KEY]')}")

    data, raw = http_get_json(url, timeout=timeout)
    return data, raw, url.replace(api_key, "[VWORLD_API_KEY]")


def call_building_use_api(pnu, api_key, domain=None, timeout=30, sleep_sec=0.05):
    if not pnu:
        return None

    params = {
        "pnu": str(pnu),
        "format": "json",
        "numOfRows": "100",
        "pageNo": "1",
        "key": api_key,
    }
    if domain:
        params["domain"] = domain

    url = build_url("https://api.vworld.kr/ned/data/getBuildingUse", params)

    try:
        data, _raw = http_get_json(url, timeout=timeout, sleep_sec=sleep_sec)
        return data
    except Exception:
        return None


def recursive_dicts(obj):
    if isinstance(obj, dict):
        yield obj
        for v in obj.values():
            yield from recursive_dicts(v)
    elif isinstance(obj, list):
        for item in obj:
            yield from recursive_dicts(item)


def best_building_use_item(api_data):
    if not api_data:
        return None

    candidates = []
    for d in recursive_dicts(api_data):
        if any(k in d for k in ["buldHg", "groundFloorCo", "buldNm", "gisIdntfcNo", "buldIdntfcNo"]):
            candidates.append(d)

    if not candidates:
        return None

    candidates.sort(key=lambda d: 0 if parse_float_first(d.get("buldHg"), None) is not None else 1)
    return candidates[0]


def extract_features(wfs_data):
    if isinstance(wfs_data, dict) and "features" in wfs_data and isinstance(wfs_data["features"], list):
        return wfs_data["features"]

    for d in recursive_dicts(wfs_data):
        if "features" in d and isinstance(d["features"], list):
            return d["features"]

    return []


def normalize_lonlat_from_ring(ring):

    out = []
    for p in ring:
        if not isinstance(p, (list, tuple)) or len(p) < 2:
            continue
        a = safe_float(p[0], None)
        b = safe_float(p[1], None)
        if a is None or b is None:
            continue

        if 30 <= a <= 45 and 120 <= b <= 135:
            lat, lon = a, b
        else:
            lon, lat = a, b
        out.append((lat, lon))

    return out


def rings_from_geojson_geometry(geom):

    if not isinstance(geom, dict):
        return []

    gtype = geom.get("type")
    coords = geom.get("coordinates")
    rings = []

    if gtype == "Polygon":
        if isinstance(coords, list) and coords:
            rings.append(normalize_lonlat_from_ring(coords[0]))
    elif gtype == "MultiPolygon":
        if isinstance(coords, list):
            for poly in coords:
                if isinstance(poly, list) and poly:
                    rings.append(normalize_lonlat_from_ring(poly[0]))
    elif gtype == "GeometryCollection":
        for g in geom.get("geometries", []):
            rings.extend(rings_from_geojson_geometry(g))

    return [ensure_closed(r) for r in rings if len(r) >= 3]

DEFAULT_HEIGHT_FIELDS = [

    "buldHg", "BULDHG", "buld_hg", "BULD_HG", "bld_hg", "BLD_HG",
    "buld_hgt", "BULD_HGT", "bld_hgt", "BLD_HGT", "height", "HEIGHT",
    "hg", "HG", "a16", "A16",

    "건물높이", "높이", "건축물높이", "건물고도",
]

DEFAULT_FLOOR_FIELDS = [
    "groundFloorCo", "GROUND_FLOOR_CO", "grnd_flr_cnt", "GRND_FLR_CNT",
    "floor_cnt", "FLOOR_CNT", "ufloor", "UFLOOR", "flr", "FLR",
    "a15", "A15", "a14", "A14", "지상층수", "층수", "층", "ground_floor",
]

DEFAULT_NAME_FIELDS = [
    "buldNm", "BULD_NM", "bld_nm", "BLD_NM", "name", "NAME", "건물명", "bd_nm", "BD_NM"
]

DEFAULT_PNU_FIELDS = [
    "pnu", "PNU", "a0", "A0", "pnu_cd", "PNU_CD", "mgt", "MGT"
]

DEFAULT_GIS_ID_FIELDS = [
    "gisIdntfcNo", "GIS_IDNTFC_NO", "gis_id", "GIS_ID",
    "buldIdntfcNo", "BLD_ID", "geoidn", "GEOIDN", "id", "ID"
]


def split_fields(text_or_list):
    if text_or_list is None:
        return []
    if isinstance(text_or_list, list):
        return text_or_list
    return [x.strip() for x in str(text_or_list).split(",") if x.strip()]


def get_first_value_ci(props, field_names):
    if not isinstance(props, dict):
        return None, ""

    lower_map = {str(k).lower(): k for k in props.keys()}
    for name in field_names:
        if name in props:
            v = props.get(name)
            if v not in (None, ""):
                return v, name
        lk = str(name).lower()
        if lk in lower_map:
            real_key = lower_map[lk]
            v = props.get(real_key)
            if v not in (None, ""):
                return v, real_key
    return None, ""


def infer_height_from_props(props, height_fields, floor_fields, floor_height_m, default_height):
    hv, hfield = get_first_value_ci(props, height_fields)
    h = parse_float_first(hv, None)
    if h is not None:

        if 1.0 <= h <= 500.0:
            return h, f"vworld_height_field:{hfield}", str(hv), ""

    fv, ffield = get_first_value_ci(props, floor_fields)
    floors = parse_float_first(fv, None)
    if floors is not None and 1 <= floors <= 200:
        return floors * floor_height_m, f"vworld_floor_field:{ffield}", "", str(fv)

    return float(default_height), "default", "", ""


def infer_height_from_building_use(item, floor_height_m):
    if not isinstance(item, dict):
        return None, "", "", ""

    h = parse_float_first(item.get("buldHg"), None)
    if h is not None and 1.0 <= h <= 500.0:
        return h, "vworld_getBuildingUse:buldHg", str(item.get("buldHg", "")), str(item.get("groundFloorCo", ""))

    floors = parse_float_first(item.get("groundFloorCo"), None)
    if floors is not None and 1 <= floors <= 200:
        return floors * floor_height_m, "vworld_getBuildingUse:groundFloorCo", "", str(item.get("groundFloorCo", ""))

    return None, "", "", ""


def estimate_height_by_area(area_m2):

    if area_m2 >= 12000:
        return 45.0, "auto_estimate_area_very_large"
    if area_m2 >= 6000:
        return 36.0, "auto_estimate_area_large"
    if area_m2 >= 2500:
        return 27.0, "auto_estimate_area_medium_large"
    if area_m2 >= 1000:
        return 21.0, "auto_estimate_area_medium"
    if area_m2 >= 350:
        return 15.0, "auto_estimate_area_small"
    return 9.0, "auto_estimate_area_tiny"

def convert_wfs_to_buildings(
    wfs_data,
    bbox,
    api_key,
    domain,
    bbox_policy="clip",
    min_area=5.0,
    default_height=15.0,
    floor_height_m=3.0,
    height_fields=None,
    floor_fields=None,
    enrich_building_use=False,
    max_attr_calls=200,
):
    center_lat = float(bbox["center_lat"])
    center_lon = float(bbox["center_lon"])

    min_lat = float(bbox["min_lat"])
    max_lat = float(bbox["max_lat"])
    min_lon = float(bbox["min_lon"])
    max_lon = float(bbox["max_lon"])

    bbox_min_x, bbox_min_y = latlon_to_xy(min_lat, min_lon, center_lat, center_lon)
    bbox_max_x, bbox_max_y = latlon_to_xy(max_lat, max_lon, center_lat, center_lon)
    min_x_limit = min(bbox_min_x, bbox_max_x)
    max_x_limit = max(bbox_min_x, bbox_max_x)
    min_y_limit = min(bbox_min_y, bbox_max_y)
    max_y_limit = max(bbox_min_y, bbox_max_y)

    height_fields = split_fields(height_fields) or DEFAULT_HEIGHT_FIELDS
    floor_fields = split_fields(floor_fields) or DEFAULT_FLOOR_FIELDS

    features = extract_features(wfs_data)

    buildings_simple = []
    buildings_footprints = []
    field_counter = Counter()
    height_source_counter = Counter()

    skipped_no_geometry = 0
    skipped_outside_bbox = 0
    skipped_small_area = 0
    attr_call_count = 0

    for feature_index, feat in enumerate(features):
        if not isinstance(feat, dict):
            continue

        props = feat.get("properties", {}) or {}
        if not isinstance(props, dict):
            props = {}

        for k in props.keys():
            field_counter[str(k)] += 1

        geom = feat.get("geometry")
        rings_latlon = rings_from_geojson_geometry(geom)
        if not rings_latlon:
            skipped_no_geometry += 1
            continue

        best = None

        for ring_latlon in rings_latlon:
            xy_points_raw = [latlon_to_xy(lat, lon, center_lat, center_lon) for lat, lon in ring_latlon]
            raw_area = polygon_area_xy(xy_points_raw)
            if raw_area <= 0:
                continue

            xs_raw = [p[0] for p in xy_points_raw]
            ys_raw = [p[1] for p in xy_points_raw]
            raw_min_x, raw_max_x = min(xs_raw), max(xs_raw)
            raw_min_y, raw_max_y = min(ys_raw), max(ys_raw)

            fully_inside = (
                raw_min_x >= min_x_limit and raw_max_x <= max_x_limit and
                raw_min_y >= min_y_limit and raw_max_y <= max_y_limit
            )

            if bbox_policy == "remove" and not fully_inside:
                continue

            if bbox_policy == "clip":
                xy_points = clip_polygon_rect(xy_points_raw, min_x_limit, max_x_limit, min_y_limit, max_y_limit)
                bbox_action = "inside" if fully_inside else "clipped"
            else:
                xy_points = ensure_closed(xy_points_raw)
                bbox_action = "inside" if fully_inside else "kept_outside"

            if len(xy_points) < 3:
                continue

            area = polygon_area_xy(xy_points)
            if area < min_area:
                continue

            if best is None or area > best["area_m2"]:
                best = {
                    "xy_points": xy_points,
                    "ring_latlon": ring_latlon,
                    "area_m2": area,
                    "raw_area_m2": raw_area,
                    "bbox_action": bbox_action,
                    "raw_min_x": raw_min_x,
                    "raw_max_x": raw_max_x,
                    "raw_min_y": raw_min_y,
                    "raw_max_y": raw_max_y,
                }

        if best is None:

            skipped_outside_bbox += 1
            continue

        xy_points = best["xy_points"]
        area_m2 = best["area_m2"]
        if area_m2 < min_area:
            skipped_small_area += 1
            continue

        xs = [p[0] for p in xy_points]
        ys = [p[1] for p in xy_points]
        min_x = min(xs)
        max_x = max(xs)
        min_y = min(ys)
        max_y = max(ys)
        width = max_x - min_x
        depth = max_y - min_y
        if width <= 0.1 or depth <= 0.1:
            skipped_small_area += 1
            continue

        cx = (min_x + max_x) / 2.0
        cy = (min_y + max_y) / 2.0

        pnu, pnu_field = get_first_value_ci(props, DEFAULT_PNU_FIELDS)
        gis_id, gis_id_field = get_first_value_ci(props, DEFAULT_GIS_ID_FIELDS)
        name, name_field = get_first_value_ci(props, DEFAULT_NAME_FIELDS)
        if name is None:
            name = ""

        height_m, height_source, VWorld_height_tag, VWorld_floor_tag = infer_height_from_props(
            props,
            height_fields=height_fields,
            floor_fields=floor_fields,
            floor_height_m=floor_height_m,
            default_height=default_height,
        )

        building_use_item = None

        if (
            enrich_building_use
            and pnu
            and attr_call_count < max_attr_calls
            and height_source == "default"
            and area_m2 >= 300.0
        ):
            attr_call_count += 1
            attr_data = call_building_use_api(pnu, api_key, domain=domain)
            building_use_item = best_building_use_item(attr_data)
            h2, src2, htag2, ftag2 = infer_height_from_building_use(building_use_item, floor_height_m)
            if h2 is not None:
                height_m = h2
                height_source = src2
                VWorld_height_tag = htag2
                VWorld_floor_tag = ftag2
                if not name and building_use_item:
                    name = building_use_item.get("buldNm", "") or building_use_item.get("buldDongNm", "") or ""

        if height_source == "default":
            height_m, height_source = estimate_height_by_area(area_m2)
            VWorld_height_tag = ""
            VWorld_floor_tag = ""

        height_source_counter[height_source] += 1

        b_id = len(buildings_simple)
        VWorld_id = gis_id or pnu or feat.get("id", f"feature_{feature_index}")

        simple_row = {
            "id": b_id,
            "VWorld_type": "vworld_wfs",
            "VWorld_id": VWorld_id,
            "osm_type": "vworld_wfs",
            "osm_id": VWorld_id,
            "x_m": f"{cx:.6f}",
            "y_m": f"{cy:.6f}",
            "width_m": f"{width:.6f}",
            "depth_m": f"{depth:.6f}",
            "height_m": f"{height_m:.3f}",
            "height_source": height_source,
            "height_auto_m": f"{height_m:.3f}",
            "height_auto_source": height_source,
            "min_x_m": f"{min_x:.6f}",
            "max_x_m": f"{max_x:.6f}",
            "min_y_m": f"{min_y:.6f}",
            "max_y_m": f"{max_y:.6f}",
            "raw_min_x_m": f"{best['raw_min_x']:.6f}",
            "raw_max_x_m": f"{best['raw_max_x']:.6f}",
            "raw_min_y_m": f"{best['raw_min_y']:.6f}",
            "raw_max_y_m": f"{best['raw_max_y']:.6f}",
            "bbox_action": best["bbox_action"],
            "area_m2": f"{area_m2:.6f}",
            "raw_area_m2": f"{best['raw_area_m2']:.6f}",
            "building_tag": "vworld_building",
            "name": str(name),
            "VWorld_height_tag": str(VWorld_height_tag),
            "VWorld_building_levels": str(VWorld_floor_tag),
            "pnu": str(pnu or ""),
            "pnu_field": pnu_field,
            "gis_id_field": gis_id_field,
            "name_field": name_field,
            "feature_id": str(feat.get("id", "")),
            "source": "vworld_wfs",
        }

        footprint_row = {
            "id": b_id,
            "VWorld_type": "vworld_wfs",
            "VWorld_id": VWorld_id,
            "osm_type": "vworld_wfs",
            "osm_id": VWorld_id,
            "height_m": f"{height_m:.3f}",
            "height_source": height_source,
            "height_auto_m": f"{height_m:.3f}",
            "height_auto_source": height_source,
            "area_m2": f"{area_m2:.6f}",
            "polygon_xy_json": json.dumps(xy_points, ensure_ascii=False),
            "polygon_latlon_json": json.dumps(best["ring_latlon"], ensure_ascii=False),
            "building_tag": "vworld_building",
            "name": str(name),
            "VWorld_height_tag": str(VWorld_height_tag),
            "VWorld_building_levels": str(VWorld_floor_tag),
            "pnu": str(pnu or ""),
            "feature_id": str(feat.get("id", "")),
            "source": "vworld_wfs",
        }

        buildings_simple.append(simple_row)
        buildings_footprints.append(footprint_row)

    stats = {
        "features_count": len(features),
        "usable_buildings_count": len(buildings_simple),
        "skipped_no_geometry": skipped_no_geometry,
        "skipped_outside_bbox_or_clipped_empty": skipped_outside_bbox,
        "skipped_small_area": skipped_small_area,
        "height_source_counter": dict(height_source_counter),
        "field_counter": dict(field_counter),
        "attr_call_count": attr_call_count,
    }

    return buildings_simple, buildings_footprints, stats


def manual_key(row):
    feature_id = str(row.get("feature_id", "") or "").strip()
    if feature_id:
        return ("feature_id", feature_id)

    VWorld_type = str(row.get("VWorld_type", "") or "").strip()
    VWorld_id = str(row.get("VWorld_id", "") or "").strip()
    name = str(row.get("name", "") or "").strip()
    return ("fallback", VWorld_type, VWorld_id, name)


def read_existing_manual_overrides(manual_path):
    manual_path = Path(manual_path)
    overrides = {}

    if not manual_path.exists():
        return overrides

    try:
        with open(manual_path, newline="", encoding="utf-8-sig") as f:
            reader = csv.DictReader(f)
            for row in reader:
                key = manual_key(row)
                overrides[key] = row
    except Exception:
        return overrides

    return overrides


def apply_manual_heights(buildings_simple, manual_overrides):
    applied = 0

    for row in buildings_simple:
        old = manual_overrides.get(manual_key(row))

        if old is None:
            continue

        manual_value = old.get("manual_height_m", "")

        h = safe_float(manual_value, None)

        if h is not None and h > 0:
            row["height_m"] = f"{h:.3f}"
            row["height_source"] = "manual_height_m"
            applied += 1

    return applied


def update_manual_csv(manual_path, buildings_simple, manual_overrides=None):
    if manual_overrides is None:
        manual_overrides = {}

    rows = []

    for row in buildings_simple:
        old = manual_overrides.get(manual_key(row), {})

        manual_height_m = old.get("manual_height_m", "")
        user_memo = old.get("user_memo", "")

        rows.append({
            "feature_id": row.get("feature_id", ""),
            "VWorld_type": row.get("VWorld_type", ""),
            "VWorld_id": row.get("VWorld_id", ""),
            "name": row.get("name", ""),
            "building_tag": row.get("building_tag", ""),
            "auto_height_m": row.get("height_auto_m", ""),
            "auto_height_source": row.get("height_auto_source", ""),
            "manual_height_m": manual_height_m,
            "final_height_m": row.get("height_m", ""),
            "final_height_source": row.get("height_source", ""),
            "area_m2": row.get("area_m2", ""),
            "width_m": row.get("width_m", ""),
            "depth_m": row.get("depth_m", ""),
            "pnu": row.get("pnu", ""),
            "user_memo": user_memo,
            "guide": "자동 높이가 틀릴 때만 manual_height_m에 원하는 높이(m)를 입력"
        })

    fieldnames = [
        "feature_id", "VWorld_type", "VWorld_id", "name", "building_tag",
        "auto_height_m", "auto_height_source", "manual_height_m",
        "final_height_m", "final_height_source", "area_m2", "width_m", "depth_m",
        "pnu", "user_memo", "guide"
    ]

    write_csv_rows(manual_path, rows, fieldnames, encoding="utf-8-sig")

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bbox", required=True, help="bbox.json path")
    parser.add_argument("--output-dir", required=True, help="output directory")

    parser.add_argument("--api-key", default=os.environ.get("VWORLD_API_KEY", ""), help="VWorld API key. 기본값: 환경변수 VWORLD_API_KEY")
    parser.add_argument("--domain", default=os.environ.get("VWORLD_DOMAIN", ""), help="VWorld key 발급 시 등록한 도메인. 예: http://localhost")
    parser.add_argument("--typename", default="lt_c_bldginfo", help="VWorld WFS typename. 기본: lt_c_bldginfo")
    parser.add_argument("--maxfeatures", type=int, default=1000)
    parser.add_argument("--output-format", default="application/json")
    parser.add_argument("--timeout", type=int, default=60)

    parser.add_argument("--bbox-policy", choices=["clip", "remove", "keep"], default="clip")
    parser.add_argument("--min-area", type=float, default=5.0)
    parser.add_argument("--default-height", type=float, default=15.0)
    parser.add_argument("--floor-height", type=float, default=3.0)
    parser.add_argument("--height-fields", default=",".join(DEFAULT_HEIGHT_FIELDS))
    parser.add_argument("--floor-fields", default=",".join(DEFAULT_FLOOR_FIELDS))

    parser.add_argument(
        "--enrich-building-use",
        action="store_true",
        help="PNU가 있을 때 /ned/data/getBuildingUse 속성 API를 추가 호출해서 buldHg/groundFloorCo 보강"
    )
    parser.add_argument("--max-attr-calls", type=int, default=200)

    args = parser.parse_args()

    bbox_path = Path(args.bbox).resolve()
    output_dir = Path(args.output_dir).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    if not args.api_key:
        raise RuntimeError(
            "VWorld API key가 없습니다. 아래처럼 먼저 등록하세요:\n"
            "export VWORLD_API_KEY='발급받은_개발키'"
        )

    bbox = read_json(bbox_path)

    wfs_data, raw_text, safe_url = call_vworld_wfs(
        bbox=bbox,
        api_key=args.api_key,
        domain=args.domain,
        typename=args.typename,
        maxfeatures=args.maxfeatures,
        output=args.output_format,
        timeout=args.timeout,
    )

    raw_json_path = output_dir / "vworld_wfs_raw.json"
    save_json(raw_json_path, wfs_data)

    buildings_simple, buildings_footprints, stats = convert_wfs_to_buildings(
        wfs_data=wfs_data,
        bbox=bbox,
        api_key=args.api_key,
        domain=args.domain,
        bbox_policy=args.bbox_policy,
        min_area=args.min_area,
        default_height=args.default_height,
        floor_height_m=args.floor_height,
        height_fields=args.height_fields,
        floor_fields=args.floor_fields,
        enrich_building_use=args.enrich_building_use,
        max_attr_calls=args.max_attr_calls,
    )

    simple_path = output_dir / "buildings.csv"
    footprint_path = output_dir / "buildings_footprints.csv"
    manual_path = output_dir / "building_heights_manual.csv"
    report_path = output_dir / "vworld_field_report.json"

    simple_fields = [
        "id", "VWorld_type", "VWorld_id", "osm_type", "osm_id", "x_m", "y_m", "width_m", "depth_m",
        "height_m", "height_source", "height_auto_m", "height_auto_source",
        "min_x_m", "max_x_m", "min_y_m", "max_y_m",
        "raw_min_x_m", "raw_max_x_m", "raw_min_y_m", "raw_max_y_m",
        "bbox_action", "area_m2", "raw_area_m2", "building_tag", "name",
        "VWorld_height_tag", "VWorld_building_levels", "pnu", "pnu_field", "gis_id_field",
        "name_field", "feature_id", "source"
    ]

    footprint_fields = [
        "id", "VWorld_type", "VWorld_id", "osm_type", "osm_id", "height_m", "height_source",
        "height_auto_m", "height_auto_source", "area_m2",
        "polygon_xy_json", "polygon_latlon_json", "building_tag", "name",
        "VWorld_height_tag", "VWorld_building_levels", "pnu", "feature_id", "source"
    ]

    manual_overrides = read_existing_manual_overrides(manual_path)
    manual_applied = apply_manual_heights(buildings_simple, manual_overrides)

    write_csv_rows(simple_path, buildings_simple, simple_fields, encoding="utf-8")
    write_csv_rows(footprint_path, buildings_footprints, footprint_fields, encoding="utf-8")
    update_manual_csv(manual_path, buildings_simple, manual_overrides)

    if manual_applied > 0:
        print(f"[INFO] manual_height_m 반영 건물 수: {manual_applied}")

    report = {
        "safe_request_url": safe_url,
        "bbox_json": str(bbox_path),
        "typename": args.typename,
        "bbox_policy": args.bbox_policy,
        "outputs": {
            "vworld_wfs_raw_json": str(raw_json_path),
            "buildings_csv": str(simple_path),
            "buildings_footprints_csv": str(footprint_path),
            "building_heights_manual_csv": str(manual_path),
        },
        "stats": stats,
        "top_fields": Counter(stats.get("field_counter", {})).most_common(200),
    }
    save_json(report_path, report)

    print("\n========== VWorld 건물 자동 매핑 완료 ==========")
    print(f"raw json                : {raw_json_path}")
    print(f"buildings.csv           : {simple_path}")
    print(f"buildings_footprints.csv: {footprint_path}")
    print(f"manual height csv       : {manual_path}")
    print(f"field report            : {report_path}")
    print("")
    print(f"WFS feature count       : {stats['features_count']}")
    print(f"usable buildings        : {stats['usable_buildings_count']}")
    print(f"skipped no geometry     : {stats['skipped_no_geometry']}")
    print(f"skipped outside/empty   : {stats['skipped_outside_bbox_or_clipped_empty']}")
    print(f"skipped small area      : {stats['skipped_small_area']}")
    print("")
    print("height_source 분포:")
    for k, v in sorted(stats["height_source_counter"].items(), key=lambda kv: (-kv[1], kv[0])):
        print(f"  {k}: {v}")

    if stats["usable_buildings_count"] == 0:
        print("\n[WARN] 건물이 0개입니다. 아래를 확인하세요.")
        print("1. 개발키가 WMS/WFS API 사용 가능 상태인지")
        print("2. --domain 값이 키 발급 시 입력한 서비스URL과 같은지")
        print("3. --typename lt_c_bldginfo 대신 lt_c_spbd 등 다른 레이어가 필요한지")
        print("4. output/vworld_wfs_raw.json에 에러 메시지가 있는지")


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print("\n[ERROR]", e, file=sys.stderr)
        sys.exit(1)
