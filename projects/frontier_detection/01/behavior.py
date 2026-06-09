import csv
import random
from math import atan2, hypot, sin, cos
from pathlib import Path

from sim_api.behavior import Behavior

from .frontier_detection import FrontierObservation
from .utils import NAV_ROUTE, ROOM_ROUTE, build_route_waypoints
from .algorithms.wfd import WavefrontFrontierDetector


DEFAULT_ROUTE_CSV = "data/routes.csv"
ROUTE_FIELD_SEPARATOR = "|"


def _clamp(value, low, high):
    return max(float(low), min(float(high), float(value)))


def _wrap_angle(angle_rad):
    return atan2(sin(float(angle_rad)), cos(float(angle_rad)))


class OfficeRouteWaypointBehavior(Behavior):
    """Visit room centers through a CSV-selected route without wall following."""

    WAYPOINTS = NAV_ROUTE

    def on_init(self, ctx):
        route_record = self._choose_route_record(ctx)
        self.selected_route_id = "default_snake"
        self.route_room_ids = [str(point["id"]) for point in ROOM_ROUTE]
        self.route_door_ids = []
        self.route_start_room_id = self.route_room_ids[0]
        self.waypoints = list(self.WAYPOINTS)

        if route_record is not None:
            self.selected_route_id = str(route_record["route_id"])
            self.route_room_ids = list(route_record["_rooms"])
            self.route_door_ids = list(route_record["_doors"])
            self.route_start_room_id = str(route_record["start_room"])
            self.waypoints = build_route_waypoints(self.route_room_ids, self.route_door_ids)
            self._set_robot_pose(
                float(route_record["start_x"]),
                float(route_record["start_y"]),
                float(route_record["start_theta"]),
            )

        self.pen.configure(color="teal", width_m=0.012, alpha=0.90).down()
        self.current_index = 0
        self.visited_count = 0
        self.total_rooms = len(self.route_room_ids)
        self.available_rooms = len(ROOM_ROUTE)
        self.total_waypoints = len(self.waypoints)
        self.current_room_id = "-"
        self.target_room_id = self.waypoints[0]["id"]
        self.target_x_m = float(self.waypoints[0]["x"])
        self.target_y_m = float(self.waypoints[0]["y"])
        self.goal_distance_m = 0.0
        self.heading_error_rad = 0.0
        self.linear_cmd = 0.0
        self.angular_cmd = 0.0
        self.frontier_candidates = []
        self.mode = "route_start"
        self._completion_reported = False
        self.detector = WavefrontFrontierDetector()
        ctx.log(
            "office route {} loaded: {} of {} rooms, {} navigation waypoints, start {}".format(
                self.selected_route_id,
                self.total_rooms,
                self.available_rooms,
                self.total_waypoints,
                self.route_start_room_id,
            ),
            source=self.robot_id,
        )

    def execute(self, ctx):
        if self.current_index >= self.total_waypoints:
            self._stop_completed(ctx)
            return

        pose = self._robot.pose
        target = self.waypoints[self.current_index]
        distance_m = self._update_target_state(pose, target)

        if distance_m <= float(self.waypoint_tolerance_m):
            if str(target.get("kind", "room")) == "room":
                self._mark_room_visited(ctx, target)
            self.current_index += 1
            if self.current_index >= self.total_waypoints:
                self._stop_completed(ctx)
                return
            target = self.waypoints[self.current_index]
            distance_m = self._update_target_state(pose, target)

        self._drive_toward_target(pose, distance_m)
        obs = self.build_frontier_observation(ctx)
        if obs is not None:
            self.frontier_candidates = self.detector.execute(obs)

    def get_frontier_candidates(self):
        """Return the latest student-produced candidates.

        The reference project does not fill this list. Student projects may use
        it as a simple handoff from behavior code to their own panel drawing
        code, but the baseline panel intentionally does not draw these points.
        """

        return list(self.frontier_candidates)

    def build_frontier_observation(self, ctx):
        """Build the standard input object expected by student detectors.

        This helper packages the robot pose and the current LiDAR scan into the
        assignment interface. A detector should call ``execute`` with this
        object and store only the candidates from the current step.
        """

        lidar_sensors = list(getattr(self._robot, "lidar_sensors", []) or [])
        if not lidar_sensors:
            return None

        lidar = lidar_sensors[0]
        samples = max(1, int(getattr(lidar, "samples", 1)))
        min_angle = float(getattr(lidar, "min_angle", -3.141592653589793))
        max_angle = float(getattr(lidar, "max_angle", 3.141592653589793))
        if samples == 1:
            angles = (0.5 * (min_angle + max_angle),)
        else:
            step = (max_angle - min_angle) / float(samples - 1)
            angles = tuple(min_angle + (index * step) for index in range(samples))

        pose = self._robot.pose
        distances = tuple(float(value) for value in lidar.read())
        return FrontierObservation(
            robot_x_m=float(pose.x),
            robot_y_m=float(pose.y),
            robot_theta_rad=float(pose.theta),
            lidar_angles_rad=angles,
            lidar_distances_m=distances,
            lidar_max_distance_m=float(getattr(lidar, "max_range", 0.0)),
            step=int(getattr(ctx.sim, "step", 0)),
            time_s=float(getattr(ctx.sim, "time", 0.0)),
        )

    def _update_target_state(self, pose, target):
        if str(target.get("kind", "room")) == "room":
            self.target_room_id = str(target["id"])
        else:
            between = target.get("between", ("", ""))
            self.target_room_id = "passage {}-{}".format(str(between[0]), str(between[1]))
        self.target_x_m = float(target["x"])
        self.target_y_m = float(target["y"])
        dx = self.target_x_m - float(pose.x)
        dy = self.target_y_m - float(pose.y)
        self.goal_distance_m = hypot(dx, dy)
        target_heading = atan2(dy, dx)
        self.heading_error_rad = _wrap_angle(target_heading - float(pose.theta))
        return self.goal_distance_m

    def _mark_room_visited(self, ctx, target):
        self.current_room_id = str(target["id"])
        self.visited_count += 1
        ctx.log(
            "visited {} center ({:.2f}, {:.2f})".format(
                self.current_room_id,
                float(target["x"]),
                float(target["y"]),
            ),
            source=self.robot_id,
        )

    def _drive_toward_target(self, pose, distance_m):
        turn = _clamp(
            float(self.turn_gain) * float(self.heading_error_rad),
            -float(self.max_angular_rad_s),
            float(self.max_angular_rad_s),
        )

        if abs(float(self.heading_error_rad)) >= float(self.rotate_in_place_angle_rad):
            linear = 0.0
            self.mode = "turn_to_waypoint"
        else:
            slowdown = _clamp(
                distance_m / max(1e-6, float(self.slowdown_distance_m)),
                0.30,
                1.0,
            )
            alignment = _clamp(
                1.0 - (abs(float(self.heading_error_rad)) / 0.75),
                0.35,
                1.0,
            )
            linear = float(self.forward_speed_m_s) * slowdown * alignment
            self.mode = "drive_to_waypoint"

        self.linear_cmd = linear
        self.angular_cmd = turn
        self.set_velocity(linear=linear, angular=turn)

    def _stop_completed(self, ctx):
        self.mode = "route_complete"
        self.linear_cmd = 0.0
        self.angular_cmd = 0.0
        self.goal_distance_m = 0.0
        self.set_velocity(linear=0.0, angular=0.0)
        if not self._completion_reported:
            self._completion_reported = True
            ctx.log(
                "route {} complete: visited {} rooms".format(
                    self.selected_route_id,
                    self.visited_count,
                ),
                source=self.robot_id,
            )
            if bool(self.stop_on_complete):
                ctx.sim.finish("route_complete")

    def _choose_route_record(self, ctx):
        records = self._load_route_records(self._route_csv_path(ctx))
        if not records:
            return None

        requested_route_id = str(getattr(self, "route_id", "random") or "random").strip()
        if requested_route_id and requested_route_id.lower() not in ("random", "any", "*"):
            for record in records:
                if str(record["route_id"]) == requested_route_id:
                    return record
            raise ValueError("Unknown route_id '{}' in routes.csv".format(requested_route_id))

        return random.choice(records)

    def _route_csv_path(self, ctx):
        raw_path = str(getattr(self, "route_csv", DEFAULT_ROUTE_CSV) or DEFAULT_ROUTE_CSV).strip()
        path = Path(raw_path)
        if not path.is_absolute():
            path = Path(ctx.project_dir) / path
        return path

    def _load_route_records(self, path):
        if not path.is_file():
            return []

        records = []
        with path.open("r", encoding="utf-8", newline="") as handle:
            for row in csv.DictReader(handle):
                room_ids = self._split_route_field(row.get("rooms", ""))
                door_ids = self._split_route_field(row.get("doors", ""))
                if not room_ids:
                    continue
                if door_ids and len(door_ids) != len(room_ids) - 1:
                    raise ValueError(
                        "Route {} has {} rooms but {} door ids".format(
                            row.get("route_id", "?"),
                            len(room_ids),
                            len(door_ids),
                        )
                    )
                record = dict(row)
                record["_rooms"] = room_ids
                record["_doors"] = door_ids
                records.append(record)
        return records

    @staticmethod
    def _split_route_field(value):
        return [
            part.strip()
            for part in str(value or "").split(ROUTE_FIELD_SEPARATOR)
            if part.strip()
        ]

    def _set_robot_pose(self, x, y, theta):
        if hasattr(self._robot, "set_initial_pose"):
            self._robot.set_initial_pose(float(x), float(y), float(theta))
        else:
            self._robot.pose.x = float(x)
            self._robot.pose.y = float(y)
            self._robot.pose.theta = float(theta)

        if hasattr(self._robot, "geometry") and hasattr(self._robot, "pose"):
            self._robot.global_geometry = self._robot.geometry.get_transformation_to_pose(self._robot.pose)
            self._robot.collision_geometry = self._robot.global_geometry

        for sensor in getattr(self._robot, "infrared_sensors", []):
            sensor.update_position()
        for lidar in getattr(self._robot, "lidar_sensors", []):
            lidar.update_position()
