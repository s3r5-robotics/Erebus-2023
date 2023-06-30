from copy import deepcopy

import numpy as np

from data_structures.vectors import Position2D
from data_structures.angle import Angle

from data_structures.compound_pixel_grid import CompoundExpandablePixelGrid
from data_structures.tile_color_grid import TileColorExpandableGrid

from mapping.wall_mapper import WallMapper
from mapping.occupied_mapping import OccupiedMapper

from mapping.array_filtering import ArrayFilterer

from mapping.robot_mapper import RobotMapper
from mapping.fixture_mapper import FixtureMapper


class Mapper:
    def __init__(self, tile_size, robot_diameter):
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
        self.pixel_grid = CompoundExpandablePixelGrid(initial_shape=np.array([1, 1]),
                                                      pixel_per_m=pixels_per_tile / self.quarter_tile_size,
                                                      robot_radius_m=(self.robot_diameter / 2) - 0.008)

        self.tile_color_grid = TileColorExpandableGrid(initial_shape=np.array((1, 1)),
                                                       tile_size=self.tile_size)

        # Data processors
        self.wall_mapper = WallMapper(self.pixel_grid, robot_diameter)

        self.occupied_mapper = OccupiedMapper(self.pixel_grid)

        self.filterer = ArrayFilterer()

        self.robot_mapper = RobotMapper(pixel_grid=self.pixel_grid,
                                        robot_diameter=self.robot_diameter,
                                        pixels_per_m=pixels_per_tile / self.quarter_tile_size)

        self.fixture_mapper = FixtureMapper(pixel_grid=self.pixel_grid,
                                            tile_size=self.tile_size)

        self.time = 0

    def update(
            self,
            in_bounds_point_cloud: list = None,
            out_of_bounds_point_cloud: list = None,
            robot_position: Position2D = None,
            robot_previous_position: Position2D = None,
            robot_orientation: Angle = None,
            time=None
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

        self.robot_mapper.map_traversed_by_robot(self.robot_grid_index)
        self.robot_mapper.map_seen_by_camera(self.robot_grid_index, self.robot_orientation)
        self.robot_mapper.map_discovered_by_robot(self.robot_grid_index, self.robot_orientation)

        self.fixture_mapper.generate_detection_zone()
        self.fixture_mapper.clean_up_fixtures()

        self.occupied_mapper.map_occupied()

        self.filterer.remove_isolated_points(self.pixel_grid)

    def register_start(self, robot_position):
        self.start_position = deepcopy(robot_position)
        print("registered start position:", self.start_position)

    def has_detected_victim_from_position(self):
        robot_array_index = self.pixel_grid.grid_index_to_array_index(self.robot_grid_index)
        return self.pixel_grid.arrays["robot_detected_fixture_from"][robot_array_index[0], robot_array_index[1]]

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


