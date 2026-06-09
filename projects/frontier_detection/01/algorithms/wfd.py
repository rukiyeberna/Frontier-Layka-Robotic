from __future__ import annotations
import math
from typing import List, Dict, Tuple
from ..frontier_detection import FrontierCandidate, FrontierDetection, FrontierObservation


class WavefrontFrontierDetector(FrontierDetection):
    """
    LiDAR tabanlı Wavefront Frontier Detector.

    Her adımda LiDAR ışınlarından local occupancy grid oluşturur,
    boş-bilinmeyen geçiş noktalarını (frontier) tespit eder,
    yakın noktaları gruplar ve her grubun merkezini döndürür.
    """

    algorithm_id = "wfd"

    # Grid parametreleri
    CELL_SIZE_M = 0.15       # Her hücre kaç metre
    GRID_RADIUS = 40         # Robot etrafında kaç hücre (her yönde)
    MIN_GROUP_SIZE = 2       # Bir frontier grubu minimum kaç nokta içermeli
    GROUP_MERGE_DIST = 0.6   # İki frontier noktası kaç metre içindeyse aynı grup

    def execute(self, observation: FrontierObservation) -> List[FrontierCandidate]:
        grid = self._build_grid(observation)
        raw_points = self._extract_frontiers(grid, observation)
        groups = self._group_frontiers(raw_points)
        return self._candidates_from_groups(groups)

    # ------------------------------------------------------------------
    # 1) Grid oluştur
    # ------------------------------------------------------------------
    def _build_grid(self, obs: FrontierObservation) -> Dict[Tuple[int,int], str]:
        """
        Sözlük tabanlı sparse grid.
        Değerler: 'free', 'occupied', 'unknown' (yoksa unknown)
        """
        grid: Dict[Tuple[int,int], str] = {}
        r = obs.robot_x_m
        c = obs.robot_y_m

        for i, (angle_rel, dist) in enumerate(
            zip(obs.lidar_angles_rad, obs.lidar_distances_m)
        ):
            angle_abs = obs.robot_theta_rad + angle_rel
            hit = dist < obs.lidar_max_distance_m * 0.99  # gerçek engel mi?

            # Ray boyunca boş hücreler
            steps = int(dist / self.CELL_SIZE_M)
            for s in range(steps):
                px = r + s * self.CELL_SIZE_M * math.cos(angle_abs)
                py = c + s * self.CELL_SIZE_M * math.sin(angle_abs)
                key = self._to_cell(px, py)
                if grid.get(key) != 'occupied':
                    grid[key] = 'free'

            # Son nokta
            end_x = r + dist * math.cos(angle_abs)
            end_y = c + dist * math.sin(angle_abs)
            end_key = self._to_cell(end_x, end_y)
            if hit:
                grid[end_key] = 'occupied'
            else:
                # Max range'e ulaştı → bilinmeyen sınır
                grid[end_key] = 'free'

        return grid

    # ------------------------------------------------------------------
    # 2) Frontier noktalarını çıkar
    # ------------------------------------------------------------------
    def _extract_frontiers(
        self, grid: Dict[Tuple[int,int], str], obs: FrontierObservation
    ) -> List[Tuple[float, float]]:
        """
        Boş bir hücrenin komşusunda 'unknown' (grid'de yok) varsa → frontier.
        """
        points = []
        for (cx, cy), state in grid.items():
            if state != 'free':
                continue
            for nx, ny in self._neighbors4(cx, cy):
                if (nx, ny) not in grid:  # unknown
                    # Frontier noktası = boş hücrenin merkezi (global koordinat)
                    wx = cx * self.CELL_SIZE_M
                    wy = cy * self.CELL_SIZE_M
                    points.append((wx, wy))
                    break
        return points

    # ------------------------------------------------------------------
    # 3) Yakın noktaları grupla
    # ------------------------------------------------------------------
    def _group_frontiers(
        self, points: List[Tuple[float, float]]
    ) -> List[List[Tuple[float, float]]]:
        if not points:
            return []

        groups: List[List[Tuple[float, float]]] = []
        used = [False] * len(points)

        for i, pt in enumerate(points):
            if used[i]:
                continue
            group = [pt]
            used[i] = True
            for j, other in enumerate(points):
                if used[j]:
                    continue
                if self._dist(pt, other) < self.GROUP_MERGE_DIST:
                    group.append(other)
                    used[j] = True
            groups.append(group)

        return [g for g in groups if len(g) >= self.MIN_GROUP_SIZE]

    # ------------------------------------------------------------------
    # 4) Her gruptan bir FrontierCandidate üret
    # ------------------------------------------------------------------
    def _candidates_from_groups(
        self, groups: List[List[Tuple[float, float]]]
    ) -> List[FrontierCandidate]:
        candidates = []
        for group in groups:
            cx = sum(p[0] for p in group) / len(group)
            cy = sum(p[1] for p in group) / len(group)
            confidence = min(1.0, len(group) / 10.0)
            candidates.append(
                FrontierCandidate(
                    x_m=cx,
                    y_m=cy,
                    label="frontier",
                    confidence=confidence,
                )
            )
        return candidates

    # ------------------------------------------------------------------
    # Yardımcılar
    # ------------------------------------------------------------------
    def _to_cell(self, x: float, y: float) -> Tuple[int, int]:
        return (
            int(math.floor(x / self.CELL_SIZE_M)),
            int(math.floor(y / self.CELL_SIZE_M)),
        )

    @staticmethod
    def _neighbors4(cx: int, cy: int):
        return [(cx+1,cy),(cx-1,cy),(cx,cy+1),(cx,cy-1)]

    @staticmethod
    def _dist(a: Tuple[float,float], b: Tuple[float,float]) -> float:
        return math.hypot(a[0]-b[0], a[1]-b[1])