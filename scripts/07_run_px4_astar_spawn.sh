#!/usr/bin/env bash
set -euo pipefail

ASTAR_DIR="/home/kang/Desktop/study/A* Algorithm"
PX4_DIR="$HOME/PX4-Autopilot"
PX4_MAKE_TARGET="${PX4_MAKE_TARGET:-px4_sitl gazebo-classic_iris__empty}"

MODE="dynamic_z"
CUSTOM_CSV=""
CUSTOM_WORLD=""

print_help() {
  cat <<'EOF'
사용법:
  ./07_run_px4_astar_spawn.sh 2d
  ./07_run_px4_astar_spawn.sh dynamic

모드:
  2d, z10, 2d_z10        -> output/path_full_2d_z10.csv 사용
  dynamic, dynamic_z, 3d -> output/path_full_dynamic_z.csv 사용
  legacy                 -> output/path_full.csv 사용

옵션:
  --mode MODE            -> 모드 지정
  --csv PATH             -> 사용할 path_full CSV 직접 지정
  --world PATH           -> 사용할 Gazebo world 직접 지정
  -h, --help             -> 도움말 출력

예시:
  ./07_run_px4_astar_spawn.sh 2d
  ./07_run_px4_astar_spawn.sh dynamic
  ./07_run_px4_astar_spawn.sh --csv ../output/path_full_dynamic_z.csv
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    2d|z10|2d_z10)
      MODE="2d_z10"
      shift
      ;;
    dynamic|dyn|dynamic_z|3d)
      MODE="dynamic_z"
      shift
      ;;
    legacy|old)
      MODE="legacy"
      shift
      ;;
    --mode)
      MODE="${2:-}"
      shift 2
      ;;
    --csv)
      CUSTOM_CSV="${2:-}"
      shift 2
      ;;
    --world)
      CUSTOM_WORLD="${2:-}"
      shift 2
      ;;
    -h|--help)
      print_help
      exit 0
      ;;
    *)
      echo "ERROR: 알 수 없는 인자입니다: $1"
      echo ""
      print_help
      exit 1
      ;;
  esac
done

case "$MODE" in
  2d|z10|2d_z10)
    MODE="2d_z10"
    DEFAULT_PATH_CSV="$ASTAR_DIR/output/path_full_2d_z10.csv"
    MODE_DESC="2D A* / z=10m fixed / buildings are obstacles"
    MODE_SPAWN_JSON="$ASTAR_DIR/output/spawn_pose_2d_z10.json"
    ;;
  dynamic|dyn|dynamic_z|3d)
    MODE="dynamic_z"
    DEFAULT_PATH_CSV="$ASTAR_DIR/output/path_full_dynamic_z.csv"
    MODE_DESC="Dynamic-Z A* / shortest path with changing altitude"
    MODE_SPAWN_JSON="$ASTAR_DIR/output/spawn_pose_dynamic_z.json"
    ;;
  legacy|old)
    MODE="legacy"
    DEFAULT_PATH_CSV="$ASTAR_DIR/output/path_full.csv"
    MODE_DESC="Legacy path_full.csv"
    MODE_SPAWN_JSON="$ASTAR_DIR/output/spawn_pose_legacy.json"
    ;;
  *)
    echo "ERROR: 지원하지 않는 MODE입니다: $MODE"
    echo "2d 또는 dynamic 중 하나를 사용하세요."
    exit 1
    ;;
esac

if [[ -n "$CUSTOM_CSV" ]]; then
  if [[ "$CUSTOM_CSV" = /* ]]; then
    PATH_CSV="$CUSTOM_CSV"
  else
    PATH_CSV="$ASTAR_DIR/scripts/$CUSTOM_CSV"
  fi
else
  PATH_CSV="$DEFAULT_PATH_CSV"
fi

META_JSON="$ASTAR_DIR/output/map_meta.json"
HEIGHTMAP="$ASTAR_DIR/output/heightmap.png"

if [[ -n "$CUSTOM_WORLD" ]]; then
  if [[ "$CUSTOM_WORLD" = /* ]]; then
    WORLD_SRC="$CUSTOM_WORLD"
  else
    WORLD_SRC="$ASTAR_DIR/scripts/$CUSTOM_WORLD"
  fi
else
  WORLD_SRC="$ASTAR_DIR/output/generated_world.world"
fi

WORLD_DST="$PX4_DIR/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/empty.world"

SPAWN_JSON="$ASTAR_DIR/output/spawn_pose.json"

SITL_RUN="$PX4_DIR/Tools/simulation/gazebo-classic/sitl_run.sh"
BACKUP="${SITL_RUN}.backup_astar_spawn"

IRIS_BASE_Z_M="0.83"
GROUND_CLEARANCE_M="1.5"
SPAWN_SAMPLE_RADIUS_M="20.0"

cd "$ASTAR_DIR/scripts" 2>/dev/null || cd "$ASTAR_DIR"

if [[ "$PATH_CSV" != /* ]]; then
  PATH_CSV="$(realpath "$PATH_CSV")"
fi

if [[ "$WORLD_SRC" != /* ]]; then
  WORLD_SRC="$(realpath "$WORLD_SRC")"
fi

echo ""
echo "=============================================="
echo " PX4 + Gazebo A* Spawn 실행"
echo "=============================================="
echo "MODE      : $MODE"
echo "MODE_DESC : $MODE_DESC"
echo "ASTAR_DIR : $ASTAR_DIR"
echo "PX4_DIR   : $PX4_DIR"
echo "PX4_TARGET: $PX4_MAKE_TARGET"
echo "PATH_CSV  : $PATH_CSV"
echo "WORLD_SRC : $WORLD_SRC"
echo "WORLD_DST : $WORLD_DST"
echo "SPAWN_JSON: $SPAWN_JSON"
echo "=============================================="
echo ""

check_file() {
  if [ ! -f "$1" ]; then
    echo "ERROR: 필요한 파일이 없습니다:"
    echo "$1"
    exit 1
  fi
}

check_file "$PATH_CSV"
check_file "$META_JSON"
check_file "$HEIGHTMAP"
check_file "$WORLD_SRC"
check_file "$SITL_RUN"

mkdir -p "$ASTAR_DIR/output"

echo "[1/7] sitl_run.sh 백업/복구"

if [ -f "$BACKUP" ]; then
  cp "$BACKUP" "$SITL_RUN"
  echo "기존 백업에서 sitl_run.sh 원본 복구 완료"
else
  cp "$SITL_RUN" "$BACKUP"
  echo "백업 생성 완료: $BACKUP"
fi

echo ""
echo "[2/7] 선택된 CSV 첫 좌표 기준 spawn 위치 계산"

read START_X START_Y START_Z TERRAIN_CENTER_Z TERRAIN_LOCAL_MAX_Z FLIGHT_Z <<EOF
$(python3 - "$PATH_CSV" "$META_JSON" "$HEIGHTMAP" "$IRIS_BASE_Z_M" "$GROUND_CLEARANCE_M" "$SPAWN_SAMPLE_RADIUS_M" <<'PY'
import csv
import json
import sys
import math
from pathlib import Path

from PIL import Image

path_csv = Path(sys.argv[1])
meta_json = Path(sys.argv[2])
heightmap_path = Path(sys.argv[3])

iris_base_z = float(sys.argv[4])
ground_clearance = float(sys.argv[5])
sample_radius_m = float(sys.argv[6])

with open(path_csv, newline="", encoding="utf-8-sig") as f:
    rows = list(csv.DictReader(f))

if not rows:
    raise RuntimeError("path CSV가 비어 있습니다.")

first = rows[0]

required = ["gazebo_x_m", "gazebo_y_m", "gazebo_z_m"]
missing = [c for c in required if c not in first]
if missing:
    raise RuntimeError(f"CSV에 필요한 컬럼이 없습니다: {missing}. 현재 컬럼: {list(first.keys())}")

x = float(first["gazebo_x_m"])
y = float(first["gazebo_y_m"])
flight_z = float(first.get("gazebo_z_m", 0.0) or 0.0)

with open(meta_json, "r", encoding="utf-8") as f:
    meta = json.load(f)

size_x_m = float(meta["gazebo_size_x_m"])
size_y_m = float(meta["gazebo_size_y_m"])
z_size_m = float(meta["gazebo_z_size_m"])

img = Image.open(heightmap_path)

if img.mode not in ("L", "I;16", "I", "F"):
    img = img.convert("L")

w, h = img.size
pixels = list(img.getdata())

mn = min(float(v) for v in pixels)
mx = max(float(v) for v in pixels)

def norm_value(v):
    v = float(v)
    if mx <= mn:
        return 0.0
    return max(0.0, min(1.0, (v - mn) / (mx - mn)))

def xy_to_pixel_float(px_m, py_m):
    col = (px_m + size_x_m / 2.0) / size_x_m * (w - 1)
    row = (size_y_m / 2.0 - py_m) / size_y_m * (h - 1)
    return row, col

def sample_z(px_m, py_m):
    row, col = xy_to_pixel_float(px_m, py_m)
    r = int(round(max(0, min(h - 1, row))))
    c = int(round(max(0, min(w - 1, col))))
    value = img.getpixel((c, r))
    if isinstance(value, tuple):
        value = value[0]
    return norm_value(value) * z_size_m

terrain_center_z = sample_z(x, y)

min_x = x - sample_radius_m
max_x = x + sample_radius_m
min_y = y - sample_radius_m
max_y = y + sample_radius_m

r1, c1 = xy_to_pixel_float(min_x, max_y)
r2, c2 = xy_to_pixel_float(max_x, min_y)

r0 = max(0, min(h - 1, int(math.floor(min(r1, r2)))))
r3 = max(0, min(h - 1, int(math.ceil(max(r1, r2)))))
c0 = max(0, min(w - 1, int(math.floor(min(c1, c2)))))
c3 = max(0, min(w - 1, int(math.ceil(max(c1, c2)))))

local_max_norm = 0.0

for rr in range(r0, r3 + 1):
    for cc in range(c0, c3 + 1):
        value = img.getpixel((cc, rr))
        if isinstance(value, tuple):
            value = value[0]
        local_max_norm = max(local_max_norm, norm_value(value))

terrain_local_max_z = local_max_norm * z_size_m
spawn_z = terrain_local_max_z + iris_base_z + ground_clearance

print(f"{x} {y} {spawn_z} {terrain_center_z} {terrain_local_max_z} {flight_z}")
PY
)
EOF

echo ""
echo "========== 계산된 Spawn 위치 =========="
echo "MODE                = $MODE"
echo "START_X             = $START_X"
echo "START_Y             = $START_Y"
echo "terrain_center_z    = $TERRAIN_CENTER_Z"
echo "terrain_local_max_z = $TERRAIN_LOCAL_MAX_Z"
echo "csv_first_flight_z  = $FLIGHT_Z"
echo "IRIS_BASE_Z_M       = $IRIS_BASE_Z_M"
echo "GROUND_CLEARANCE_M  = $GROUND_CLEARANCE_M"
echo "FINAL START_Z       = $START_Z"
echo "======================================="
echo ""

echo "[3/7] spawn_pose.json 저장"

python3 - "$SPAWN_JSON" "$MODE_SPAWN_JSON" "$MODE" "$MODE_DESC" "$PATH_CSV" "$START_X" "$START_Y" "$START_Z" "$TERRAIN_CENTER_Z" "$TERRAIN_LOCAL_MAX_Z" "$FLIGHT_Z" "$IRIS_BASE_Z_M" "$GROUND_CLEARANCE_M" "$SPAWN_SAMPLE_RADIUS_M" <<'PY'
import json
import sys
from pathlib import Path

common_out = Path(sys.argv[1])
mode_out = Path(sys.argv[2])

mode = sys.argv[3]
mode_desc = sys.argv[4]
path_csv = sys.argv[5]

data = {
    "mode": mode,
    "mode_desc": mode_desc,
    "path_csv": path_csv,
    "spawn_x_m": float(sys.argv[6]),
    "spawn_y_m": float(sys.argv[7]),
    "spawn_z_m": float(sys.argv[8]),
    "terrain_center_z_m": float(sys.argv[9]),
    "terrain_local_max_z_m": float(sys.argv[10]),
    "csv_first_flight_z_m": float(sys.argv[11]),
    "iris_base_z_m": float(sys.argv[12]),
    "ground_clearance_m": float(sys.argv[13]),
    "spawn_sample_radius_m": float(sys.argv[14]),
    "coordinate_note": "Gazebo world spawn pose. 08_csv_waypoint_follower.py subtracts this pose from the selected path CSV for MAVROS local path tracking."
}

for out in (common_out, mode_out):
    out.parent.mkdir(parents=True, exist_ok=True)
    with open(out, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, ensure_ascii=False)
PY

echo "공통 저장 완료: $SPAWN_JSON"
echo "모드별 저장 완료: $MODE_SPAWN_JSON"
echo ""
echo "========== spawn_pose.json =========="
cat "$SPAWN_JSON"
echo ""
echo "====================================="
echo ""

echo "[4/7] generated_world.world를 PX4 empty.world로 복사"

cp "$WORLD_SRC" "$WORLD_DST"

echo "복사 완료:"
echo "$WORLD_SRC"
echo " -> "
echo "$WORLD_DST"
echo ""

echo "[5/7] sitl_run.sh spawn line 수정"

python3 - "$SITL_RUN" "$START_X" "$START_Y" "$START_Z" <<'PY'
import re
import sys
from pathlib import Path

path = Path(sys.argv[1])
x = float(sys.argv[2])
y = float(sys.argv[3])
z = float(sys.argv[4])

text = path.read_text(encoding="utf-8", errors="replace")
lines = text.splitlines(keepends=True)

out = []
changed = 0

for line in lines:
    original = line

    if "gz model" in line and "--spawn-file" in line:
        # 과거에 잘못 붙은 '; do -x ... -y ... -z ...' 형태 제거
        line = re.sub(
            r";\s*do\s+-x\s+[^ \t\n]+?\s+-y\s+[^ \t\n]+?\s+-z\s+[^ \t\n]+",
            "; do",
            line
        )

        # 이미 -x -y -z가 있는 경우 교체
        line2, n = re.subn(
            r"-x\s+[^ \t\n\\]+\s+-y\s+[^ \t\n\\]+\s+-z\s+[^ \t\n\\]+",
            f"-x {x:.6f} -y {y:.6f} -z {z:.6f}",
            line,
            count=1
        )

        # -x -y -z가 없는 경우 2>&1 앞에 삽입
        if n == 0:
            line2, n = re.subn(
                r"(\s+2>&1\s*\|)",
                lambda m: f" -x {x:.6f} -y {y:.6f} -z {z:.6f}" + m.group(1),
                line,
                count=1
            )

        if n > 0:
            changed += 1
            line = line2

            print("")
            print("========== spawn line 수정 ==========")
            print("[기존]")
            print(original.rstrip())
            print("[수정]")
            print(line.rstrip())
            print("====================================")

    out.append(line)

if changed == 0:
    print("ERROR: gz model --spawn-file 줄을 찾지 못했습니다.")
    sys.exit(1)

path.write_text("".join(out), encoding="utf-8")
print(f"수정 완료: {changed}개 spawn line")
PY

echo ""
echo "========== 현재 sitl_run.sh spawn line =========="
grep -n "gz model.*spawn-file" "$SITL_RUN" || true
echo "==============================================="
echo ""

echo "[6/7] 기존 PX4/Gazebo 프로세스 종료"

pkill -9 -x px4 || true
pkill -9 -x gazebo || true
pkill -9 -x gzserver || true
pkill -9 -x gzclient || true

echo "종료 완료"
echo ""

echo "[7/7] PX4 Gazebo 실행"

cd "$PX4_DIR"

unset GAZEBO_MASTER_URI
unset GAZEBO_IP

export GAZEBO_MASTER_URI=http://127.0.0.1:11345
export GAZEBO_IP=127.0.0.1
export GAZEBO_MODEL_PATH="$HOME/.gazebo/models:$PX4_DIR/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models:${GAZEBO_MODEL_PATH:-}"

if ! make $PX4_MAKE_TARGET; then
  echo ""
  echo "[WARN] PX4_MAKE_TARGET='$PX4_MAKE_TARGET' 실행 실패"
  echo "[WARN] fallback: make px4_sitl gazebo"
  make px4_sitl gazebo
fi
