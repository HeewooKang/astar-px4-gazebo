#!/usr/bin/env python3
import argparse
import csv
import json
import math
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode


def clamp(value, min_value, max_value):
    return max(min_value, min(max_value, value))


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


def read_spawn_pose(spawn_path):
    spawn_path = Path(spawn_path).expanduser().resolve()

    if not spawn_path.exists():
        return None

    with open(spawn_path, "r", encoding="utf-8") as f:
        data = json.load(f)

    def get_value(*keys):
        for key in keys:
            if key in data:
                return float(data[key])
        raise KeyError(f"spawn_pose.json에서 키를 찾지 못했습니다: {keys}")

    return {
        "x": get_value("spawn_x_m", "START_X", "start_x"),
        "y": get_value("spawn_y_m", "START_Y", "start_y"),
        "z": get_value("spawn_z_m", "START_Z", "start_z"),
        "source": str(spawn_path),
    }


def resolve_z_mode(args, csv_path, rows, fieldnames):

    if args.z_mode != "auto":
        return args.z_mode

    name = csv_path.name.lower()

    if "2d" in name or "z10" in name or "fixed" in name:
        return "relative"

    if "dynamic" in name or "dyn" in name or "3d" in name:
        return "absolute"

    z_values = [safe_float(row.get("gazebo_z_m"), 0.0) for row in rows]
    z_min = min(z_values)
    z_max = max(z_values)
    z_range = z_max - z_min

    if z_range < 1e-6 and 0.0 <= z_max <= 50.0:
        return "relative"

    dynamic_columns = {
        "terrain_z_m",
        "nearby_building_top_z_m",
        "building_clearance_m",
        "altitude_reason",
    }

    if any(col in dynamic_columns for col in fieldnames):
        return "absolute"

    return "absolute"


def load_waypoints(csv_path, args):
    csv_path = Path(csv_path).expanduser().resolve()

    if not csv_path.exists():
        raise FileNotFoundError(f"waypoint CSV 파일이 없습니다: {csv_path}")

    raw_rows = []

    with open(csv_path, newline="", encoding="utf-8-sig") as f:
        reader = csv.DictReader(f)

        required = ["gazebo_x_m", "gazebo_y_m", "gazebo_z_m"]

        for col in required:
            if col not in reader.fieldnames:
                raise RuntimeError(
                    f"CSV에 '{col}' 컬럼이 없습니다. 현재 컬럼: {reader.fieldnames}"
                )

        fieldnames = list(reader.fieldnames or [])

        for row in reader:
            raw_rows.append(row)

    if len(raw_rows) == 0:
        raise RuntimeError("waypoint가 0개입니다. A* GUI에서 path_full CSV를 먼저 생성하세요.")

    spawn = None
    coordinate_mode = "gazebo_world_absolute_no_spawn_offset"

    if args.use_spawn_offset:
        spawn = read_spawn_pose(args.spawn_pose)

        if spawn is None:
            first = raw_rows[0]
            spawn = {
                "x": safe_float(first.get("gazebo_x_m")),
                "y": safe_float(first.get("gazebo_y_m")),
                "z": safe_float(first.get("terrain_z_m"), 0.0),
                "source": "fallback:first_waypoint_xy_and_terrain_z",
            }

            print("")
            print("[WARN] spawn_pose.json을 찾지 못했습니다.")
            print(f"[WARN] 요청 경로: {Path(args.spawn_pose).expanduser().resolve()}")
            print("[WARN] 첫 waypoint 기준으로 임시 local 변환합니다.")
            print("[WARN] 정확한 z 보정을 위해 PX4 실행 스크립트에서 output/spawn_pose.json을 저장하는 것을 권장합니다.")
            print("")

        coordinate_mode = "gazebo_xy_to_mavros_local"

    z_mode = resolve_z_mode(args, csv_path, raw_rows, fieldnames)

    waypoints = []
    debug_rows = []

    for row in raw_rows:
        world_x = safe_float(row.get("gazebo_x_m"))
        world_y = safe_float(row.get("gazebo_y_m"))
        world_z = safe_float(row.get("gazebo_z_m"))

        if args.use_spawn_offset:
            x = world_x - spawn["x"]
            y = world_y - spawn["y"]

            if z_mode == "relative":
                z = world_z
            elif z_mode == "absolute":
                z = world_z - spawn["z"]
            else:
                raise RuntimeError(f"알 수 없는 z_mode: {z_mode}")
        else:
            x = world_x
            y = world_y
            z = world_z

        waypoints.append((x, y, z))

        debug_rows.append({
            "world_x": world_x,
            "world_y": world_y,
            "world_z": world_z,
            "local_x": x,
            "local_y": y,
            "local_z": z,
        })

    info = {
        "csv_path": str(csv_path),
        "coordinate_mode": coordinate_mode,
        "z_mode": z_mode,
        "spawn": spawn,
        "debug_rows": debug_rows,
        "fieldnames": fieldnames,
    }

    return waypoints, info


class CsvWaypointFollower(Node):
    def __init__(self, args):
        super().__init__("csv_waypoint_follower")

        self.args = args
        self.waypoints, self.path_info = load_waypoints(args.csv, args)

        self.current_pose = None
        self.current_state = State()

        self.current_index = 0
        self.finished = False

        self.setpoint_count = 0

        self.offboard_confirmed = False
        self.arm_confirmed = False
        self.last_offboard_request_time = None
        self.last_arm_request_time = None

        self.home_xy_initialized = False
        self.home_x = 0.0
        self.home_y = 0.0
        self.takeoff_done = False

        pose_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.pose_sub = self.create_subscription(
            PoseStamped,
            "/mavros/local_position/pose",
            self.pose_callback,
            pose_qos
        )

        self.state_sub = self.create_subscription(
            State,
            "/mavros/state",
            self.state_callback,
            10
        )

        self.vel_pub = self.create_publisher(
            TwistStamped,
            args.topic,
            10
        )

        self.arming_client = self.create_client(
            CommandBool,
            "/mavros/cmd/arming"
        )

        self.mode_client = self.create_client(
            SetMode,
            "/mavros/set_mode"
        )

        self.timer = self.create_timer(1.0 / args.rate, self.control_loop)

        self.print_startup_info()

    def print_startup_info(self):
        first = self.waypoints[0]
        last = self.waypoints[-1]

        dbg_first = self.path_info["debug_rows"][0]
        dbg_last = self.path_info["debug_rows"][-1]

        local_z_values = [p[2] for p in self.waypoints]

        self.get_logger().info("")
        self.get_logger().info("========== CSV Waypoint Follower 시작 ==========")
        self.get_logger().info(f"CSV path        : {self.path_info['csv_path']}")
        self.get_logger().info(f"Waypoint count  : {len(self.waypoints)}")
        self.get_logger().info(f"Coordinate mode : {self.path_info['coordinate_mode']}")
        self.get_logger().info(f"Z mode          : {self.path_info['z_mode']}")

        spawn = self.path_info.get("spawn")

        if spawn is not None:
            self.get_logger().info(
                f"Spawn offset    : x={spawn['x']:.3f}, y={spawn['y']:.3f}, z={spawn['z']:.3f}"
            )
            self.get_logger().info(f"Spawn source    : {spawn['source']}")

        self.get_logger().info(f"Velocity topic  : {self.args.topic}")
        self.get_logger().info("MAVROS frame    : run `ros2 param set /mavros/setpoint_velocity mav_frame LOCAL_NED` before flight")
        self.get_logger().info(f"Rate            : {self.args.rate} Hz")
        self.get_logger().info(f"Max XY speed    : {self.args.max_xy_speed} m/s")
        self.get_logger().info(f"Max Z speed     : {self.args.max_z_speed} m/s")
        self.get_logger().info(f"Accept radius   : {self.args.accept_radius} m")
        self.get_logger().info(f"Takeoff first   : {self.args.takeoff_first}")
        self.get_logger().info(f"Takeoff altitude: {self.args.takeoff_altitude} m")
        self.get_logger().info(f"Auto OFFBOARD   : {self.args.auto_offboard}")
        self.get_logger().info(f"Auto ARM        : {self.args.auto_arm}")
        self.get_logger().info(f"Preflight sec   : {self.args.preflight_setpoint_seconds}")
        self.get_logger().info(f"Retry interval  : {self.args.request_interval}")
        self.get_logger().info("")

        self.get_logger().info(
            f"FIRST target(local): x={first[0]:.2f}, y={first[1]:.2f}, z={first[2]:.2f}"
        )
        self.get_logger().info(
            f"LAST  target(local): x={last[0]:.2f}, y={last[1]:.2f}, z={last[2]:.2f}"
        )
        self.get_logger().info(
            f"Local Z range      : min={min(local_z_values):.2f}, max={max(local_z_values):.2f}"
        )
        self.get_logger().info(
            f"FIRST world csv    : x={dbg_first['world_x']:.2f}, "
            f"y={dbg_first['world_y']:.2f}, z={dbg_first['world_z']:.2f}"
        )
        self.get_logger().info(
            f"LAST  world csv    : x={dbg_last['world_x']:.2f}, "
            f"y={dbg_last['world_y']:.2f}, z={dbg_last['world_z']:.2f}"
        )
        self.get_logger().info("")

        if min(local_z_values) <= 1.0:
            self.get_logger().warn(
                "local target z가 1m 이하인 waypoint가 있습니다. "
                "2D z=10 경로라면 --z-mode relative를 사용해야 합니다."
            )

        if self.args.use_spawn_offset:
            self.get_logger().info(
                "spawn offset 적용됨: Gazebo CSV 좌표를 MAVROS local 좌표로 변환해서 추종합니다."
            )
        else:
            self.get_logger().warn(
                "spawn offset 미적용: CSV 좌표를 그대로 추종합니다. START 위치 스폰이면 경로가 밀릴 수 있습니다."
            )

        self.get_logger().info("")

    def pose_callback(self, msg):
        self.current_pose = msg

        if not self.home_xy_initialized:
            self.home_x = msg.pose.position.x
            self.home_y = msg.pose.position.y
            self.home_xy_initialized = True

    def state_callback(self, msg):
        self.current_state = msg

    def distance_to_target(self, target):
        if self.current_pose is None:
            return float("inf"), float("inf"), float("inf"), 0.0, 0.0, 0.0

        px = self.current_pose.pose.position.x
        py = self.current_pose.pose.position.y
        pz = self.current_pose.pose.position.z

        tx, ty, tz = target

        dx = tx - px
        dy = ty - py
        dz = tz - pz

        dist_xy = math.sqrt(dx * dx + dy * dy)
        dist_3d = math.sqrt(dx * dx + dy * dy + dz * dz)

        return dist_3d, dist_xy, abs(dz), dx, dy, dz

    def request_offboard(self):
        if not self.mode_client.service_is_ready():
            self.get_logger().warn("OFFBOARD service not ready: /mavros/set_mode")
            return

        req = SetMode.Request()
        req.custom_mode = "OFFBOARD"

        future = self.mode_client.call_async(req)
        future.add_done_callback(self.offboard_response)

    def offboard_response(self, future):
        try:
            result = future.result()
            self.get_logger().info(f"OFFBOARD 요청 결과: mode_sent={result.mode_sent}")
        except Exception as e:
            self.get_logger().warn(f"OFFBOARD 요청 실패: {e}")

    def request_arm(self):
        if not self.arming_client.service_is_ready():
            self.get_logger().warn("ARM service not ready: /mavros/cmd/arming")
            return

        req = CommandBool.Request()
        req.value = True

        future = self.arming_client.call_async(req)
        future.add_done_callback(self.arm_response)

    def arm_response(self, future):
        try:
            result = future.result()
            self.get_logger().info(f"ARM 요청 결과: success={result.success}")
        except Exception as e:
            self.get_logger().warn(f"ARM 요청 실패: {e}")

    def publish_velocity(self, vx, vy, vz):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"

        msg.twist.linear.x = float(vx)
        msg.twist.linear.y = float(vy)
        msg.twist.linear.z = float(vz)

        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = 0.0

        self.vel_pub.publish(msg)
        self.setpoint_count += 1

    def publish_stop(self):
        self.publish_velocity(0.0, 0.0, 0.0)

    def maybe_request_mode_and_arm(self):
        if self.setpoint_count < self.args.rate * self.args.preflight_setpoint_seconds:
            return

        if hasattr(self.current_state, "connected") and not self.current_state.connected:
            if self.setpoint_count % int(max(1, self.args.rate)) == 0:
                self.get_logger().warn("MAVROS FCU 미연결: /mavros/state connected=False")
            return

        now_sec = self.get_clock().now().nanoseconds / 1e9

        if self.args.auto_offboard:
            if self.current_state.mode == "OFFBOARD":
                if not self.offboard_confirmed:
                    self.offboard_confirmed = True
                    self.get_logger().info("OFFBOARD 모드 확인 완료")
            else:
                if (
                    self.last_offboard_request_time is None
                    or now_sec - self.last_offboard_request_time >= self.args.request_interval
                ):
                    self.last_offboard_request_time = now_sec
                    self.get_logger().info(
                        f"OFFBOARD 모드 요청: current_mode={self.current_state.mode}"
                    )
                    self.request_offboard()

        if self.args.auto_arm:
            if self.current_state.armed:
                if not self.arm_confirmed:
                    self.arm_confirmed = True
                    self.get_logger().info("ARM 확인 완료")
            else:
                if (
                    self.last_arm_request_time is None
                    or now_sec - self.last_arm_request_time >= self.args.request_interval
                ):
                    self.last_arm_request_time = now_sec
                    self.get_logger().info("ARM 요청")
                    self.request_arm()

    def velocity_to_target(self, target):
        dist_3d, dist_xy, dist_z, dx, dy, dz = self.distance_to_target(target)

        vx = self.args.kp_xy * dx
        vy = self.args.kp_xy * dy

        xy_speed = math.sqrt(vx * vx + vy * vy)

        if xy_speed > self.args.max_xy_speed:
            scale = self.args.max_xy_speed / xy_speed
            vx *= scale
            vy *= scale

        vz = self.args.kp_z * dz
        vz = clamp(vz, -self.args.max_z_speed, self.args.max_z_speed)

        if dist_3d < self.args.slow_radius:
            slow_scale = max(0.25, dist_3d / self.args.slow_radius)
            vx *= slow_scale
            vy *= slow_scale
            vz *= slow_scale

        return vx, vy, vz, dist_3d, dist_xy, dist_z

    def handle_takeoff_first(self):
        if not self.args.takeoff_first or self.takeoff_done:
            return False

        first_z = self.waypoints[0][2]
        takeoff_z = max(self.args.takeoff_altitude, first_z)
        target = (self.home_x, self.home_y, takeoff_z)

        vx, vy, vz, dist_3d, dist_xy, dist_z = self.velocity_to_target(target)
        self.publish_velocity(vx, vy, vz)

        if self.setpoint_count % int(max(1, self.args.rate)) == 0:
            px = self.current_pose.pose.position.x
            py = self.current_pose.pose.position.y
            pz = self.current_pose.pose.position.z
            self.get_logger().info(
                f"[TAKEOFF] pos=({px:.1f},{py:.1f},{pz:.1f}) "
                f"target=({target[0]:.1f},{target[1]:.1f},{target[2]:.1f}) "
                f"dist_z={dist_z:.1f} vel=({vx:.1f},{vy:.1f},{vz:.1f}) "
                f"mode={self.current_state.mode} armed={self.current_state.armed}"
            )

        if dist_z <= self.args.takeoff_accept_radius:
            self.takeoff_done = True
            self.get_logger().info(
                f"Takeoff 단계 완료: 현재 z가 목표 고도 근처입니다. path 추종 시작."
            )

        return True

    def control_loop(self):
        if self.current_pose is None:
            self.publish_stop()

            if self.setpoint_count % int(max(1, self.args.rate)) == 0:
                self.get_logger().info("현재 위치 대기 중: /mavros/local_position/pose")

            return

        self.maybe_request_mode_and_arm()

        if self.finished:
            self.publish_stop()
            return

        if self.handle_takeoff_first():
            return

        if self.current_index >= len(self.waypoints):
            self.finished = True
            self.publish_stop()
            self.get_logger().info("모든 waypoint 도착 완료")
            return

        target = self.waypoints[self.current_index]

        dist_3d, dist_xy, dist_z, dx, dy, dz = self.distance_to_target(target)

        if dist_3d <= self.args.accept_radius:
            self.get_logger().info(
                f"Waypoint {self.current_index + 1}/{len(self.waypoints)} 도착 "
                f"dist={dist_3d:.2f} m"
            )

            self.current_index += 1

            if self.current_index >= len(self.waypoints):
                self.finished = True
                self.publish_stop()
                self.get_logger().info("경로 추종 완료")
                return

            target = self.waypoints[self.current_index]

        vx, vy, vz, dist_3d, dist_xy, dist_z = self.velocity_to_target(target)
        self.publish_velocity(vx, vy, vz)

        if self.setpoint_count % int(max(1, self.args.rate)) == 0:
            px = self.current_pose.pose.position.x
            py = self.current_pose.pose.position.y
            pz = self.current_pose.pose.position.z

            self.get_logger().info(
                f"[{self.current_index + 1}/{len(self.waypoints)}] "
                f"pos=({px:.1f},{py:.1f},{pz:.1f}) "
                f"target=({target[0]:.1f},{target[1]:.1f},{target[2]:.1f}) "
                f"dist={dist_3d:.1f} "
                f"vel=({vx:.1f},{vy:.1f},{vz:.1f}) "
                f"mode={self.current_state.mode} armed={self.current_state.armed}"
            )


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--csv",
        default="../output/path_full_dynamic_z.csv",
        help="A* GUI가 만든 비행용 waypoint CSV. 예: ../output/path_full_2d_z10.csv 또는 ../output/path_full_dynamic_z.csv"
    )
    parser.add_argument("--topic", default="/mavros/setpoint_velocity/cmd_vel")

    parser.add_argument("--rate", type=float, default=20.0)
    parser.add_argument("--max-xy-speed", type=float, default=5.0)
    parser.add_argument("--max-z-speed", type=float, default=2.0)
    parser.add_argument("--accept-radius", type=float, default=4.0)
    parser.add_argument("--slow-radius", type=float, default=12.0)

    parser.add_argument("--kp-xy", type=float, default=0.6)
    parser.add_argument("--kp-z", type=float, default=0.8)

    parser.add_argument("--auto-offboard", dest="auto_offboard", action="store_true", default=True)
    parser.add_argument("--no-auto-offboard", dest="auto_offboard", action="store_false")
    parser.add_argument("--auto-arm", dest="auto_arm", action="store_true", default=True)
    parser.add_argument("--no-auto-arm", dest="auto_arm", action="store_false")

    parser.add_argument(
        "--preflight-setpoint-seconds",
        type=float,
        default=3.0,
        help="OFFBOARD/ARM 요청 전 setpoint를 먼저 보내는 시간"
    )
    parser.add_argument(
        "--request-interval",
        type=float,
        default=2.0,
        help="OFFBOARD/ARM 실패 시 재요청 간격"
    )

    parser.add_argument(
        "--spawn-pose",
        default="../output/spawn_pose.json",
        help="PX4 실행 스크립트가 저장한 spawn_pose.json"
    )

    parser.add_argument(
        "--use-spawn-offset",
        dest="use_spawn_offset",
        action="store_true",
        default=True,
        help="Gazebo CSV x/y 좌표를 MAVROS local 좌표로 자동 변환"
    )

    parser.add_argument(
        "--no-use-spawn-offset",
        dest="use_spawn_offset",
        action="store_false",
        help="spawn offset 변환을 끄고 CSV 좌표를 그대로 사용"
    )

    parser.add_argument(
        "--z-mode",
        choices=["auto", "relative", "absolute"],
        default="auto",
        help=(
            "relative: CSV z를 MAVROS local z로 그대로 사용, 2D z=10용. "
            "absolute: CSV z에서 spawn_z를 빼서 사용, dynamic-Z용. "
            "auto: 파일명/컬럼 기준 자동 판단."
        )
    )

    parser.add_argument(
        "--takeoff-first",
        dest="takeoff_first",
        action="store_true",
        default=True,
        help="경로 추종 전에 먼저 제자리 이륙 고도를 확보"
    )
    parser.add_argument(
        "--no-takeoff-first",
        dest="takeoff_first",
        action="store_false",
        help="제자리 이륙 단계를 끔"
    )
    parser.add_argument(
        "--takeoff-altitude",
        type=float,
        default=10.0,
        help="경로 추종 전 먼저 확보할 MAVROS local z 고도"
    )
    parser.add_argument(
        "--takeoff-accept-radius",
        type=float,
        default=1.0,
        help="takeoff 단계 완료로 판단할 z 오차"
    )

    args = parser.parse_args()

    rclpy.init()

    node = CsvWaypointFollower(args)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("사용자 종료 Ctrl+C")
    finally:
        node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
