from copy import deepcopy
from typing import Optional

import numpy as np

from data_structures.vectors import Position2D
from data_structures.angle import Angle

from mapping.mixed_grid import MixedGrid

from mapping.filter_array import ArrayFilterer

from mapping.wall_mapping import WallMapper
from mapping.untraversable_mapping import UntraversableMapper
from mapping.fixture_mapping import FixtureMapper


class Mapper:
    def __init__(self, tile_size: int, robot_diameter: float):
        self.tile_size = tile_size
        self.quarter_tile_size = tile_size / 2
        self.robot_diameter = robot_diameter

        self.robot_position = None
        self.robot_previous_position = None
        self.robot_orientation = None
        self.start_position = None

        self.robot_grid_index = None

        # Data structures
        pixels_per_tile = 10
        self.pixel_grid = MixedGrid(initial_shape=np.array([1, 1]),
                                    pixel_per_m=pixels_per_tile / self.quarter_tile_size)

        # Data processors
        self.wall_mapper = WallMapper(self.pixel_grid, robot_diameter)
        self.untraversable_mapper = UntraversableMapper(self.pixel_grid)
        self.fixture_mapper = FixtureMapper(pixel_grid=self.pixel_grid,
                                            tile_size=self.tile_size)

        self.filterer = ArrayFilterer()

        self.time = 0

    def __call__(
            self,
            in_bounds_point_cloud: list = None,
            out_of_bounds_point_cloud: list = None,
            robot_position: Position2D = None,
            robot_previous_position: Position2D = None,
            robot_orientation: Angle = None,
            time: Optional[int] = None
    ):
        if time is not None:
            self.time = time

        if robot_position is None or robot_orientation is None:
            return

        self.robot_position = robot_position
        self.robot_previous_position = robot_previous_position
        self.robot_orientation = robot_orientation

        self.robot_grid_index = self.pixel_grid.coordinates_to_grid_index(self.robot_position)

        # Load walls and obstacles (Lidar detections)
        if in_bounds_point_cloud is not None and out_of_bounds_point_cloud is not None:
            self.wall_mapper.load_point_cloud(in_bounds_point_cloud, out_of_bounds_point_cloud, robot_position)

        self.fixture_mapper.generate_detection_zone()
        self.fixture_mapper.clean_up_fixtures()

        self.untraversable_mapper.map_occupied()

        self.filterer.remove_isolated_points(self.pixel_grid)

    def register_start(self, robot_position):
        self.start_position = deepcopy(robot_position)
        print("registered start position:", self.start_position)

    @property
    def has_detected_victim_from_position(self):
        robot_array_index = self.pixel_grid.grid_index_to_array_index(self.robot_grid_index)
        return self.pixel_grid.arrays["robot_detected_fixture_from"][robot_array_index[0], robot_array_index[1]]

    @property
    def is_close_to_swamp(self):
        if self.robot_grid_index is None:
            return False

        swamp_check_area = 0.02

        swamp_check_area_px = round(swamp_check_area * self.pixel_grid.resolution)

        robot_array_index = self.pixel_grid.grid_index_to_array_index(self.robot_grid_index)

        min_x = max(robot_array_index[0] - swamp_check_area_px, 0)
        max_x = min(robot_array_index[0] + swamp_check_area_px, self.pixel_grid.array_shape[0])
        min_y = max(robot_array_index[1] - swamp_check_area_px, 0)
        max_y = min(robot_array_index[1] + swamp_check_area_px, self.pixel_grid.array_shape[1])

        return np.any(self.pixel_grid.arrays["swamps"][min_x:max_x, min_y:max_y])


