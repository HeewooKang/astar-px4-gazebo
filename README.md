# Terrain-Based A* Path Planning with PX4, Gazebo Classic, and MAVROS

This project implements a terrain-based UAV path planning pipeline using the A* algorithm.

The pipeline uses:

* OpenTopography Copernicus GLO-30 DEM data
* Gazebo Classic heightmap terrain
* VWorld building data
* 2D A* path planning with fixed altitude
* 3D Dynamic-Z A* path planning
* PX4 SITL
* MAVROS OFFBOARD control

The purpose of this project is to generate a realistic terrain and building environment, plan a UAV path using A*, spawn a PX4 drone at the start point, and follow the generated waypoint path in Gazebo.

---

## Demo Videos

### 2D A* Fixed-Z Mode

[Watch 2D A* demo](assets/2D_Astar_Algorithm_demo.mp4)

### 3D Dynamic-Z A* Mode

[Watch 3D Dynamic-Z A* demo](assets/3D_Astar_Algorithm_demo.mp4)

---

## Project Structure

```bash
A* Algorithm/
├── data/
│   └── input_map.tif
├── output/
└── scripts/
    ├── 01_read_tif_bbox.py
    ├── 02_make_heightmap_from_dem.py
    ├── 03_make_gazebo_8bit_heightmap.py
    ├── 04_download_vworld_buildings.py
    ├── 05_make_gazebo_world.py
    ├── 06_astar_gui_2d_z10.py
    ├── 06_astar_gui_3d_z.py
    ├── 07_run_px4_astar_spawn.sh
    └── 08_csv_waypoint_follower.py
```

`data/input_map.tif` and `output/` are not included in this repository.

---

## Pipeline Overview

```text
1. Prepare input_map.tif
2. Extract GeoTIFF bounding box
3. Generate heightmap from DEM
4. Convert heightmap for Gazebo Classic
5. Download VWorld building data
6. Generate Gazebo world
7. Run 2D A* or 3D Dynamic-Z A*
8. Launch PX4 SITL with the generated world
9. Launch MAVROS
10. Set MAVROS frame to LOCAL_NED
11. Run CSV waypoint follower
```

---

## Step 1. Prepare DEM Input

Place the OpenTopography Copernicus GLO-30 DEM file in:

```bash
data/input_map.tif
```

This file is used as the original terrain elevation data.

---

## Step 2. Extract GeoTIFF Bounding Box

```bash
cd "/home/kang/Desktop/study/A* Algorithm/scripts"

DEM_TIF="/home/kang/Desktop/study/A* Algorithm/data/input_map.tif"

PYTHONNOUSERSITE=1 /usr/bin/python3 01_read_tif_bbox.py \
  --input "$DEM_TIF" \
  --output ../output/bbox.json
```

Output:

```bash
output/bbox.json
```

This script reads the GeoTIFF coordinate reference system and extracts the latitude/longitude boundary of the map.

---

## Step 3. Generate Heightmap from DEM

```bash
cd "/home/kang/Desktop/study/A* Algorithm"

DEM_TIF="/home/kang/Desktop/study/A* Algorithm/data/input_map.tif"

PYTHONNOUSERSITE=1 /usr/bin/python3 scripts/02_make_heightmap_from_dem.py \
  --dem "$DEM_TIF" \
  --bbox output/bbox.json \
  --output-dir output \
  --size 513 \
  --vertical-exaggeration 1 \
  --min-z-size 20 \
  --force-z-size 30
```

Outputs:

```bash
output/heightmap.png
output/heightmap_preview.png
output/map_meta.json
```

This script converts the DEM into a normalized heightmap and creates metadata for Gazebo coordinate conversion.

---

## Step 4. Convert Heightmap for Gazebo Classic

```bash
cd "/home/kang/Desktop/study/A* Algorithm"

PYTHONNOUSERSITE=1 /usr/bin/python3 scripts/03_make_gazebo_8bit_heightmap.py \
  --heightmap output/heightmap.png \
  --meta output/map_meta.json
```

This script converts the generated heightmap into an 8-bit grayscale PNG that Gazebo Classic can load.

Gazebo heightmap model path:

```bash
~/.gazebo/models/opentopo_heightmap/
```

---

## Step 5. Download VWorld Building Data

Set the VWorld API key:

```bash
export VWORLD_API_KEY="YOUR_VWORLD_API_KEY"
```

Run:

```bash
cd "/home/kang/Desktop/study/A* Algorithm/scripts"

PYTHONNOUSERSITE=1 /usr/bin/python3 04_download_vworld_buildings.py \
  --bbox ../output/bbox.json \
  --output-dir ../output \
  --bbox-policy clip
```

Outputs:

```bash
output/buildings.csv
output/buildings_footprints.csv
output/building_heights_manual.csv
output/vworld_field_report.json
```

This script downloads building footprint data from VWorld and converts it into Gazebo world coordinates.

---

## Step 6. Generate Gazebo World

```bash
cd "/home/kang/Desktop/study/A* Algorithm/scripts"

PYTHONNOUSERSITE=1 /usr/bin/python3 05_make_gazebo_world.py \
  --heightmap ../output/heightmap.png \
  --meta ../output/map_meta.json \
  --buildings ../output/buildings.csv \
  --output ../output/generated_world.world \
  --base-mode max
```

Output:

```bash
output/generated_world.world
```

This script creates a Gazebo Classic world containing the DEM terrain and building models.

---

# Step 7. A* Path Planning

## Option A. 2D A* with Fixed Altitude

```bash
cd "/home/kang/Desktop/study/A* Algorithm/scripts"

PYTHONNOUSERSITE=1 /usr/bin/python3 06_astar_gui_2d_z10.py \
  --meta ../output/map_meta.json \
  --heightmap ../output/heightmap.png \
  --preview ../output/heightmap_preview.png \
  --buildings ../output/buildings.csv \
  --output ../output/path_full_2d_z10.csv \
  --output-simple ../output/path_2d_z10.csv \
  --fixed-z 10 \
  --resolution 5 \
  --safety-margin 5 \
  --landing-final-z 0 \
  --landing-step-z 1
```

Outputs:

```bash
output/path_full_2d_z10.csv
output/path_2d_z10.csv
```

In this mode:

* A* runs on a 2D grid.
* Buildings are treated as obstacles.
* The UAV altitude is fixed at z = 10 m.

---

## Option B. 3D Dynamic-Z A*

```bash
cd "/home/kang/Desktop/study/A* Algorithm/scripts"

PYTHONNOUSERSITE=1 /usr/bin/python3 06_astar_gui_3d_z.py \
  --meta ../output/map_meta.json \
  --heightmap ../output/heightmap.png \
  --preview ../output/heightmap_preview.png \
  --buildings ../output/buildings.csv \
  --output ../output/path_full_dynamic_z.csv \
  --output-simple ../output/path_dynamic_z.csv \
  --resolution 5 \
  --safety-margin 5 \
  --terrain-clearance 10 \
  --building-clearance 10 \
  --z-step 5 \
  --z-cost-weight 1.0
```

Outputs:

```bash
output/path_full_dynamic_z.csv
output/path_dynamic_z.csv
```

In this mode:

* A* runs in 3D state space.
* The UAV can change altitude.
* Terrain clearance and building clearance are considered.
* The UAV can choose between flying over buildings or avoiding them horizontally.

Important parameter:

```bash
--z-cost-weight
```

Meaning:

```text
Low value  -> altitude change is cheap -> UAV tends to fly over buildings
High value -> altitude change is expensive -> UAV tends to go around buildings
```

---

# PX4 and MAVROS Simulation

## Step 8. Run PX4 SITL with A* Spawn Position

For 2D mode:

```bash
cd "/home/kang/Desktop/study/A* Algorithm/scripts"
./07_run_px4_astar_spawn.sh 2d
```

For Dynamic-Z mode:

```bash
cd "/home/kang/Desktop/study/A* Algorithm/scripts"
./07_run_px4_astar_spawn.sh dynamic
```

This script:

* Selects the correct path CSV.
* Reads the first waypoint.
* Computes a safe spawn position.
* Saves `output/spawn_pose.json`.
* Copies the generated Gazebo world to the PX4 Gazebo Classic world directory.
* Launches PX4 SITL.

---

## Step 9. Launch MAVROS

Open a new terminal:

```bash
source /opt/ros/humble/setup.bash

ros2 launch mavros px4.launch \
  fcu_url:=udp://:14540@127.0.0.1:14557
```

Check MAVROS state:

```bash
source /opt/ros/humble/setup.bash
ros2 topic echo /mavros/state --once
```

---

## Step 10. Set LOCAL_NED Frame

```bash
source /opt/ros/humble/setup.bash
ros2 param set /mavros/setpoint_velocity mav_frame LOCAL_NED
```

---

## Step 11. Run CSV Waypoint Follower

For 2D mode:

```bash
cd "/home/kang/Desktop/study/A* Algorithm/scripts"

python3 08_csv_waypoint_follower.py \
  --csv ../output/path_2d_z10.csv \
  --spawn-pose ../output/spawn_pose.json
```

For Dynamic-Z mode:

```bash
cd "/home/kang/Desktop/study/A* Algorithm/scripts"

python3 08_csv_waypoint_follower.py \
  --csv ../output/path_dynamic_z.csv \
  --spawn-pose ../output/spawn_pose.json
```

This script:

* Loads the selected waypoint CSV.
* Reads the spawn pose.
* Converts Gazebo world coordinates into MAVROS local coordinates.
* Sends velocity commands to the UAV.
* Requests OFFBOARD mode.
* Arms the vehicle.
* Follows the path until the final waypoint.

---

## Manual MAVROS Commands

If automatic OFFBOARD or arming does not work, use:

```bash
source /opt/ros/humble/setup.bash
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'OFFBOARD'}"
```

```bash
source /opt/ros/humble/setup.bash
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"
```

Check current position:

```bash
source /opt/ros/humble/setup.bash
ros2 topic echo /mavros/local_position/pose
```

Check velocity command:

```bash
source /opt/ros/humble/setup.bash
ros2 topic echo /mavros/setpoint_velocity/cmd_vel
```

Check actual local velocity:

```bash
source /opt/ros/humble/setup.bash
ros2 topic echo /mavros/local_position/velocity_local
```

---

## 2D vs 3D Mode Comparison

| Mode            | State Space       | Building Handling                  | Altitude         |
| --------------- | ----------------- | ---------------------------------- | ---------------- |
| 2D Fixed-Z A*   | row, col          | Buildings are obstacles            | Fixed z = 10 m   |
| 3D Dynamic-Z A* | row, col, z-layer | Buildings are altitude constraints | Dynamic altitude |

---

## Notes

* `data/input_map.tif` is not included because it can be large.
* `output/` files are generated by the scripts.
* The VWorld API key must not be committed to GitHub.
* Use `YOUR_VWORLD_API_KEY` in documentation.
* PX4, Gazebo Classic, ROS2 Humble, and MAVROS must be installed before running the simulation.

---

## Author

Heewoo Kang
