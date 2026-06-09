from .utils import DOORS, NAV_ROUTE, ROOM_ROUTE


class OfficeRouteSimulationPanel:
    """Draw room-center waypoints and the active route over the map."""

    def __init__(self, ctx, config):
        self._ctx = ctx
        self._config = dict(config or {})

    def draw_underlay(self, ctx, frame):
        route_points = [[float(point["x"]), float(point["y"])] for point in self._active_route(ctx)]
        if len(route_points) >= 2:
            frame.add_lines(
                [[route_points[index], route_points[index + 1]] for index in range(len(route_points) - 1)],
                linewidth=0.010,
                color="royalblue",
                alpha=0.42,
                pattern="dashed",
            )

        for door in DOORS:
            x = float(door["x"])
            y = float(door["y"])
            half_width = float(door.get("width", 0.8)) * 0.5
            if str(door["wall"]) == "vertical":
                lines = [[[x, y - half_width], [x, y + half_width]]]
            else:
                lines = [[[x - half_width, y], [x + half_width, y]]]
            frame.add_lines(lines, linewidth=0.018, color="white", alpha=0.85)

        for room in ROOM_ROUTE:
            x = float(room["x"])
            y = float(room["y"])
            frame.add_circle([x, y], 0.026, color="royalblue", alpha=0.64)
            frame.add_text(
                [x, y],
                str(room["id"]),
                color="navy",
                alpha=0.88,
                size_px=11,
                bold=True,
                dx_px=8,
                dy_px=-8,
                bg_color="white",
                bg_alpha=0.66,
                padding_px=2,
            )

    def _active_route(self, ctx):
        robot = ctx.robots.first()
        behavior = getattr(robot, "behavior", None) if robot is not None else None
        waypoints = getattr(behavior, "waypoints", None) if behavior is not None else None
        return list(waypoints or NAV_ROUTE)

    def draw_overlay(self, ctx, frame):
        robot = ctx.robots.first()
        if robot is None:
            return
        behavior = getattr(robot, "behavior", None)
        if behavior is None:
            return

        target_x = float(getattr(behavior, "target_x_m", 0.0))
        target_y = float(getattr(behavior, "target_y_m", 0.0))
        frame.add_circle([target_x, target_y], 0.034, color="orange", alpha=0.45)
        frame.add_lines(
            [[[target_x - 0.060, target_y], [target_x + 0.060, target_y]]],
            linewidth=0.008,
            color="orange",
            alpha=0.88,
        )
        frame.add_lines(
            [[[target_x, target_y - 0.060], [target_x, target_y + 0.060]]],
            linewidth=0.008,
            color="orange",
            alpha=0.88,
        )
