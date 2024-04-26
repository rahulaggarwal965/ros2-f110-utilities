from pathlib import Path
from typing import List, Optional, Union
from array import array as Array

import numpy as np
import yaml
from geometry_msgs.msg import Point, Pose
from nav_msgs.msg import OccupancyGrid as OccupancyGridMsg
from PIL import Image
from scipy.ndimage import binary_dilation, generate_binary_structure, iterate_structure

class OccupancyGrid():
    def __init__(
        self,
        frame: str,
        resolution: float,
        dim: List[int],
        origin: Optional[Union[List[int], np.ndarray]] = None,
        grid: Optional[np.ndarray] = None
    ):

        self.frame = frame
        self.resolution = resolution
        self.dim = dim

        if grid is None:
            self.grid = np.zeros(dim[::-1], dtype=bool)
        else:
            self.grid = grid.astype(bool)

        if origin is None:
            self.origin = np.array([0, -self.dim[1] / 2 * self.resolution])
        else:
            self.origin = np.asarray(origin)

    def to_msg(self) -> OccupancyGridMsg:
        occ_grid_msg = OccupancyGridMsg()
        occ_grid_msg.header.frame_id = self.frame
        occ_grid_msg.info.resolution = self.resolution
        occ_grid_msg.info.width = self.dim[0]
        occ_grid_msg.info.height = self.dim[1]
        occ_grid_msg.info.origin = Pose(position=Point(x=self.origin[0], y=self.origin[1]))
        occ_grid_msg.data = Array("b", (self.grid.astype(np.int8) * 127).tobytes())
        return occ_grid_msg

    def transform_points(self, points: np.ndarray) -> np.ndarray:
        transformed_points = ((points - self.origin) / self.resolution).astype(int)
        dim_mask = (transformed_points[:, 0] >= 0) & (transformed_points[:, 0] < self.dim[0]) & (transformed_points[:, 1] >= 0) & (transformed_points[:, 1] < self.dim[1])
        in_grid_points = transformed_points[dim_mask]
        return in_grid_points

    def transform_indices(self, indices: np.ndarray) -> np.ndarray:
        return np.fliplr(indices) * self.resolution + self.origin

    def collision(self, point1: np.ndarray, point2: np.ndarray, collision_threshold: float = 50.0):
        num_points = int(np.ceil(np.linalg.norm(point2 - point1)) / self.resolution)
        to_check_points = np.linspace(point1, point2, num_points)
        to_check_points_grid = self.transform_points(to_check_points)
        return np.any(self.grid[to_check_points_grid[:, 1], to_check_points_grid[:, 0]] > collision_threshold)

    def reset_grid(self):
        self.grid[:, :] = 0

    def dilate(self, size: int = 3):
        struct = iterate_structure(generate_binary_structure(2, 1), iterations=size // 2)
        self.grid = binary_dilation(self.grid, struct)

    def __sub__(self, occupancy_grid: "OccupancyGrid") -> "OccupancyGrid":
        # NOTE(rahul): technically we should assert the frames/resolution/dim/origin are the same, but for performance, we omit this check
        return OccupancyGrid(
            frame=self.frame,
            resolution=self.resolution,
            dim=self.dim,
            origin=self.origin,
            grid=self.grid & ~occupancy_grid.grid
        )


    @staticmethod
    def from_map(map_file: str) -> "OccupancyGrid":
        map_image_file = Path(map_file).absolute()
        map_yaml_file = Path(f"{map_image_file.parent / map_image_file.stem}.yaml")

        with map_yaml_file.open() as file:
            map_params = yaml.safe_load(file)

        grid = np.where(np.asarray(Image.open(map_image_file).transpose(Image.FLIP_TOP_BOTTOM)) > 0, False, True)

        occupancy_grid = OccupancyGrid(
                frame="map",
                resolution=map_params["resolution"],
                dim=grid.shape[::-1],
                origin=map_params["origin"][0:2],
                grid=grid
        )

        return occupancy_grid
