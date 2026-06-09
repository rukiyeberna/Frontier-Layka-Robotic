"""Student-facing frontier detection interface for the office-floor project.

This file is part of the assignment contract. It intentionally contains no
ready-made frontier detection algorithm. Student implementations should live
under ``algorithms/`` and subclass ``FrontierDetection``.

Important: ``FrontierDetection.execute(...)`` must use only the current LiDAR
observation passed to it and return only the candidates detected for that
current step. Do not accumulate old markers inside the detector. If a project UI
draws these candidates, it should redraw the latest candidate list every frame
instead of keeping historical points on the panel.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from math import cos, sin
from typing import List, Optional, Sequence, Tuple


WorldPoint = Tuple[float, float]


@dataclass(frozen=True)
class FrontierObservation:
    """One LiDAR scan with the robot pose needed for global coordinates.

    The observation is deliberately LiDAR-based, not occupancy-grid-based. A
    detector may build its own temporary map internally if its method needs one,
    but the assignment interface does not require every solution to use a grid.
    Angles are relative to the robot heading and distances are the current LiDAR
    readings in meters.
    """

    # Robot pose in the global map frame.
    robot_x_m: float
    robot_y_m: float
    robot_theta_rad: float

    # Current LiDAR scan. Both tuples must have the same length.
    lidar_angles_rad: Tuple[float, ...]
    lidar_distances_m: Tuple[float, ...]
    lidar_max_distance_m: float

    # Simulation timestamp for algorithms that want to keep their own state.
    step: int = 0
    time_s: float = 0.0

    def __post_init__(self) -> None:
        if len(self.lidar_angles_rad) != len(self.lidar_distances_m):
            raise ValueError("lidar_angles_rad and lidar_distances_m must have the same length")

    def ray_endpoint(self, index: int) -> WorldPoint:
        """Return the global endpoint of one LiDAR ray.

        This helper is useful when turning a local LiDAR hit or max-range ray
        into a point that can be drawn on the global simulation panel.
        """

        angle = float(self.robot_theta_rad) + float(self.lidar_angles_rad[index])
        distance = float(self.lidar_distances_m[index])
        return (
            float(self.robot_x_m) + distance * cos(angle),
            float(self.robot_y_m) + distance * sin(angle),
        )


@dataclass(frozen=True)
class FrontierCandidate:
    """A candidate frontier or doorway location in global metric coordinates.

    ``x_m`` and ``y_m`` are the point that should be shown on the map if the
    student chooses to visualize the result. ``confidence`` is optional because
    many frontier methods produce a geometric candidate but no meaningful score.
    """

    x_m: float
    y_m: float
    label: str = "frontier"
    confidence: Optional[float] = None

    def as_point(self) -> WorldPoint:
        """Return the candidate as a plain ``(x_m, y_m)`` tuple."""

        return (float(self.x_m), float(self.y_m))


class FrontierDetection(ABC):
    """Base class for student frontier detection algorithms.

    Subclasses should be placed in the project's ``algorithms/`` package. The
    method receives exactly one current LiDAR observation and should return the
    current step's candidates only. Panel drawing is intentionally outside this
    class; students should decide how to visualize the returned points in their
    own project UI code.
    """

    algorithm_id = "frontier_detection"

    @abstractmethod
    def execute(self, observation: FrontierObservation) -> List[FrontierCandidate]:
        """Return candidates detected from the current LiDAR observation only."""


def candidate_points(candidates: Sequence[FrontierCandidate]) -> List[WorldPoint]:
    """Return global ``(x_m, y_m)`` points from a candidate list."""

    return [candidate.as_point() for candidate in list(candidates or [])]
