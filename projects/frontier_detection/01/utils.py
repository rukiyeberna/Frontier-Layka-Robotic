"""Route metadata and helpers for the fixed frontier-detection office map."""

OFFICE_MIN_X = -16.0
OFFICE_MIN_Y = -8.4

GRID_COLS = 10
GRID_ROWS = 4

COLUMN_WIDTHS_M = [2.1, 3.8, 2.4, 4.7, 2.2, 3.4, 5.0, 2.6, 3.2, 2.6]
ROW_HEIGHTS_M = [3.8, 5.1, 3.6, 4.9]
DOOR_WAYPOINT_STANDOFF_M = 0.60


def _bounds(start, sizes):
    values = [float(start)]
    for size in sizes:
        values.append(values[-1] + float(size))
    return values


COLUMN_BOUNDS = _bounds(OFFICE_MIN_X, COLUMN_WIDTHS_M)
ROW_BOUNDS = _bounds(OFFICE_MIN_Y, ROW_HEIGHTS_M)


def _room_id(row, col):
    if int(row) % 2 == 0:
        route_index = (int(row) * GRID_COLS) + int(col) + 1
    else:
        route_index = (int(row) * GRID_COLS) + (GRID_COLS - int(col))
    return "R{:02d}".format(route_index)


def _room_center(x_min, x_max, y_min, y_max):
    return (float(x_min) + float(x_max)) * 0.5, (float(y_min) + float(y_max)) * 0.5


def _build_rooms():
    rooms = []
    for row in range(GRID_ROWS):
        for col in range(GRID_COLS):
            x_min = float(COLUMN_BOUNDS[col])
            x_max = float(COLUMN_BOUNDS[col + 1])
            y_min = float(ROW_BOUNDS[row])
            y_max = float(ROW_BOUNDS[row + 1])
            center_x, center_y = _room_center(x_min, x_max, y_min, y_max)
            rooms.append(
                {
                    "id": _room_id(row, col),
                    "row": row,
                    "col": col,
                    "x_min": x_min,
                    "x_max": x_max,
                    "y_min": y_min,
                    "y_max": y_max,
                    "x": center_x,
                    "y": center_y,
                    "area_m2": (x_max - x_min) * (y_max - y_min),
                }
            )
    return sorted(rooms, key=lambda item: int(item["id"][1:]))


ROOMS = _build_rooms()
ROOM_BY_ID = {room["id"]: room for room in ROOMS}
ROOM_ROUTE = [
    {
        "id": room["id"],
        "x": room["x"],
        "y": room["y"],
        "row": room["row"],
        "col": room["col"],
        "area_m2": room["area_m2"],
    }
    for room in ROOMS
]


DOOR_RECORDS = [
    ("D_R01_R02_1", "R01", "R02", "vertical", -13.90000, -7.05100, 0.72),
    ("D_R02_R03_1", "R02", "R03", "vertical", -10.10000, -6.97056, 0.86),
    ("D_R02_R03_2", "R02", "R03", "vertical", -10.10000, -6.01720, 0.80),
    ("D_R03_R04_1", "R03", "R04", "vertical", -7.70000, -6.09328, 0.98),
    ("D_R04_R05_1", "R04", "R05", "vertical", -3.00000, -6.91752, 1.12),
    ("D_R04_R05_2", "R04", "R05", "vertical", -3.00000, -6.03760, 0.90),
    ("D_R05_R06_1", "R05", "R06", "vertical", -0.80000, -5.88940, 0.80),
    ("D_R06_R07_1", "R06", "R07", "vertical", 2.60000, -6.84452, 1.04),
    ("D_R07_R08_1", "R07", "R08", "vertical", 7.60000, -6.28240, 0.90),
    ("D_R08_R09_1", "R08", "R09", "vertical", 10.20000, -6.61552, 0.76),
    ("D_R09_R10_1", "R09", "R10", "vertical", 13.40000, -7.05100, 0.72),
    ("D_R20_R19_1", "R20", "R19", "vertical", -13.90000, -2.05000, 0.86),
    ("D_R19_R18_1", "R19", "R18", "vertical", -10.10000, -2.71708, 0.98),
    ("D_R19_R18_2", "R19", "R18", "vertical", -10.10000, -1.39516, 1.04),
    ("D_R18_R17_1", "R18", "R17", "vertical", -7.70000, -2.38804, 1.12),
    ("D_R17_R16_1", "R17", "R16", "vertical", -3.00000, -2.75380, 0.80),
    ("D_R17_R16_2", "R17", "R16", "vertical", -3.00000, -1.33804, 0.76),
    ("D_R16_R15_1", "R16", "R15", "vertical", -0.80000, -2.57002, 1.04),
    ("D_R15_R14_1", "R15", "R14", "vertical", 2.60000, -2.73340, 0.90),
    ("D_R15_R14_2", "R15", "R14", "vertical", 2.60000, -1.35844, 0.86),
    ("D_R14_R13_1", "R14", "R13", "vertical", 7.60000, -2.21752, 0.76),
    ("D_R13_R12_1", "R13", "R12", "vertical", 10.20000, -2.76400, 0.72),
    ("D_R13_R12_2", "R13", "R12", "vertical", 10.20000, -1.41148, 1.12),
    ("D_R12_R11_1", "R12", "R11", "vertical", 13.40000, -2.05000, 0.86),
    ("D_R21_R22_1", "R21", "R22", "vertical", -13.90000, 2.67572, 0.98),
    ("D_R22_R23_1", "R22", "R23", "vertical", -10.10000, 2.09696, 1.12),
    ("D_R23_R24_1", "R23", "R24", "vertical", -7.70000, 1.85120, 0.80),
    ("D_R23_R24_2", "R23", "R24", "vertical", -7.70000, 2.75696, 0.76),
    ("D_R24_R25_1", "R24", "R25", "vertical", -3.00000, 1.98248, 1.04),
    ("D_R25_R26_1", "R25", "R26", "vertical", -0.80000, 2.50160, 0.90),
    ("D_R26_R27_1", "R26", "R27", "vertical", 2.60000, 2.19248, 0.76),
    ("D_R27_R28_1", "R27", "R28", "vertical", 7.60000, 1.84100, 0.72),
    ("D_R27_R28_2", "R27", "R28", "vertical", 7.60000, 2.68352, 1.12),
    ("D_R28_R29_1", "R28", "R29", "vertical", 10.20000, 2.30000, 0.86),
    ("D_R29_R30_1", "R29", "R30", "vertical", 13.40000, 2.67572, 0.98),
    ("D_R40_R39_1", "R40", "R39", "vertical", -13.90000, 6.22996, 1.12),
    ("D_R39_R38_1", "R39", "R38", "vertical", -10.10000, 7.39710, 0.80),
    ("D_R38_R37_1", "R38", "R37", "vertical", -7.70000, 6.05698, 1.04),
    ("D_R37_R36_1", "R37", "R36", "vertical", -3.00000, 6.85560, 0.90),
    ("D_R36_R35_1", "R36", "R35", "vertical", -0.80000, 6.39048, 0.76),
    ("D_R35_R34_1", "R35", "R34", "vertical", 2.60000, 5.79000, 0.72),
    ("D_R34_R33_1", "R34", "R33", "vertical", 7.60000, 6.55000, 0.86),
    ("D_R33_R32_1", "R33", "R32", "vertical", 10.20000, 7.12722, 0.98),
    ("D_R32_R31_1", "R32", "R31", "vertical", 13.40000, 6.22996, 1.12),
    ("D_R01_R20_1", "R01", "R20", "horizontal", -15.17800, -4.60000, 0.72),
    ("D_R02_R19_1", "R02", "R19", "horizontal", -12.00000, -4.60000, 0.86),
    ("D_R03_R18_1", "R03", "R18", "horizontal", -8.71028, -4.60000, 0.98),
    ("D_R04_R17_1", "R04", "R17", "horizontal", -5.65204, -4.60000, 1.12),
    ("D_R05_R16_1", "R05", "R16", "horizontal", -1.63340, -4.60000, 0.80),
    ("D_R06_R15_1", "R06", "R15", "horizontal", 0.60948, -4.60000, 1.04),
    ("D_R07_R14_1", "R07", "R14", "horizontal", 5.41360, -4.60000, 0.90),
    ("D_R08_R13_1", "R08", "R13", "horizontal", 8.83248, -4.60000, 0.76),
    ("D_R09_R12_1", "R09", "R12", "horizontal", 11.36300, -4.60000, 0.72),
    ("D_R10_R11_1", "R10", "R11", "horizontal", 14.70000, -4.60000, 0.86),
    ("D_R20_R21_1", "R20", "R21", "horizontal", -14.80678, 0.50000, 0.98),
    ("D_R19_R22_1", "R19", "R22", "horizontal", -12.22104, 0.50000, 1.12),
    ("D_R18_R23_1", "R18", "R23", "horizontal", -8.59040, 0.50000, 0.80),
    ("D_R17_R24_1", "R17", "R24", "horizontal", -5.81602, 0.50000, 1.04),
    ("D_R16_R25_1", "R16", "R25", "horizontal", -1.81040, 0.50000, 0.90),
    ("D_R15_R26_1", "R15", "R26", "horizontal", 0.80048, 0.50000, 0.76),
    ("D_R14_R27_1", "R14", "R27", "horizontal", 4.32100, 0.50000, 0.72),
    ("D_R13_R28_1", "R13", "R28", "horizontal", 8.90000, 0.50000, 0.86),
    ("D_R12_R29_1", "R12", "R29", "horizontal", 12.11372, 0.50000, 0.98),
    ("D_R11_R30_1", "R11", "R30", "horizontal", 14.58696, 0.50000, 1.12),
    ("D_R21_R40_1", "R21", "R40", "horizontal", -14.70490, 4.10000, 0.80),
    ("D_R22_R39_1", "R22", "R39", "horizontal", -12.34452, 4.10000, 1.04),
    ("D_R23_R38_1", "R23", "R38", "horizontal", -8.79440, 4.10000, 0.90),
    ("D_R24_R37_1", "R24", "R37", "horizontal", -5.50152, 4.10000, 0.76),
    ("D_R25_R36_1", "R25", "R36", "horizontal", -2.14700, 4.10000, 0.72),
    ("D_R26_R35_1", "R26", "R35", "horizontal", 0.90000, 4.10000, 0.86),
    ("D_R27_R34_1", "R27", "R34", "horizontal", 5.69272, 4.10000, 0.98),
    ("D_R28_R33_1", "R28", "R33", "horizontal", 8.78696, 4.10000, 1.12),
    ("D_R29_R32_1", "R29", "R32", "horizontal", 12.28160, 4.10000, 0.80),
    ("D_R30_R31_1", "R30", "R31", "horizontal", 14.51748, 4.10000, 1.04),
]


DOORS = [
    {
        "id": door_id,
        "between": (room_a, room_b),
        "wall": wall,
        "x": float(x),
        "y": float(y),
        "width": float(width),
    }
    for door_id, room_a, room_b, wall, x, y, width in DOOR_RECORDS
]

DOORS_BY_PAIR = {}
for _door in DOORS:
    DOORS_BY_PAIR.setdefault(tuple(sorted(_door["between"])), []).append(_door)


def _pair_key(room_a, room_b):
    return tuple(sorted([str(room_a["id"]), str(room_b["id"])]))


def _transition_door(room_a, room_b):
    doors = list(DOORS_BY_PAIR.get(_pair_key(room_a, room_b), []))
    if not doors:
        raise ValueError("No door between {} and {}".format(room_a["id"], room_b["id"]))
    ax, ay = float(room_a["x"]), float(room_a["y"])
    bx, by = float(room_b["x"]), float(room_b["y"])
    mid_x = (ax + bx) * 0.5
    mid_y = (ay + by) * 0.5
    return min(
        doors,
        key=lambda door: ((float(door["x"]) - mid_x) ** 2) + ((float(door["y"]) - mid_y) ** 2),
    )


def transition_door(room_a, room_b, door_id=None):
    doors = list(DOORS_BY_PAIR.get(_pair_key(room_a, room_b), []))
    if not doors:
        raise ValueError("No door between {} and {}".format(room_a["id"], room_b["id"]))
    if door_id:
        for door in doors:
            if str(door["id"]) == str(door_id):
                return door
        raise ValueError(
            "Door {} is not between {} and {}".format(
                door_id,
                room_a["id"],
                room_b["id"],
            )
        )
    return _transition_door(room_a, room_b)


def _near_door_waypoint(door, room):
    x = float(door["x"])
    y = float(door["y"])
    room_x = float(room["x"])
    room_y = float(room["y"])
    standoff = float(DOOR_WAYPOINT_STANDOFF_M)

    if str(door["wall"]) == "vertical":
        x += -standoff if room_x < x else standoff
    else:
        y += -standoff if room_y < y else standoff

    return x, y


def _transition_waypoint(door, room, phase):
    x, y = _near_door_waypoint(door, room)
    return {
        "kind": "passage",
        "id": "{}_{}_{}".format(door["id"], str(room["id"]), str(phase)),
        "x": float(x),
        "y": float(y),
        "between": door["between"],
        "door_id": door["id"],
        "room_id": room["id"],
        "phase": str(phase),
    }


def build_route_waypoints(room_ids, door_ids=None):
    room_ids = [str(room_id).strip() for room_id in list(room_ids or []) if str(room_id).strip()]
    door_ids = [str(door_id).strip() for door_id in list(door_ids or []) if str(door_id).strip()]
    if not room_ids:
        raise ValueError("A navigation route requires at least one room")
    if door_ids and len(door_ids) != len(room_ids) - 1:
        raise ValueError(
            "Route with {} rooms requires {} door ids, got {}".format(
                len(room_ids),
                len(room_ids) - 1,
                len(door_ids),
            )
        )

    waypoints = []
    for index, room_id in enumerate(room_ids):
        room = ROOM_BY_ID[str(room_id)]
        waypoints.append({"kind": "room", "id": room["id"], "x": float(room["x"]), "y": float(room["y"])})
        if index + 1 >= len(room_ids):
            continue
        current_room = ROOM_BY_ID[room["id"]]
        target_room = ROOM_BY_ID[str(room_ids[index + 1])]
        door_id = door_ids[index] if door_ids else None
        door = transition_door(current_room, target_room, door_id=door_id)
        waypoints.append(_transition_waypoint(door, current_room, "approach"))
        waypoints.append(_transition_waypoint(door, target_room, "exit"))
    return waypoints


NAV_ROUTE = build_route_waypoints([room["id"] for room in ROOM_ROUTE])
