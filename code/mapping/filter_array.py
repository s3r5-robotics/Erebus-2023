from flow.timing_alignment import TimingAlignment

import numpy as np
import cv2 as cv


class ArrayFilterer:
    def __init__(self) -> None:
        self.isolated_point_remover_kernel = np.array([[-2, -2, -2],
                                                       [-2, 1, -2],
                                                       [-2, -2, -2]])

        self.jagged_edge_remover_kernel = np.array([[0, -1, 0],
                                                    [-1, 3, -1],
                                                    [0, -1, 0]])

        self.missing_point_filler_kernel = np.array([[0, 1, 0],
                                                     [1, 0, 1],
                                                     [0, 1, 0]])

        self.isolated_point_timing = TimingAlignment(100)
        self.jagged_edge_timing = TimingAlignment(100)

    def remove_isolated_points(self, pixel_grid) -> np.ndarray:
        if self.isolated_point_timing.check:
            isolated_points_mask = cv.filter2D(pixel_grid.arrays["occupied"].astype(np.uint8), -1,
                                               self.isolated_point_remover_kernel) > 0
            pixel_grid.arrays["occupied"][isolated_points_mask] = False
            pixel_grid.arrays["walls"][isolated_points_mask] = False
            pixel_grid.arrays["holes"][isolated_points_mask] = False
            pixel_grid.arrays["detected_points"][isolated_points_mask] = 0

        self.isolated_point_timing.increase()

    def smooth_edges(self, array: np.ndarray) -> np.ndarray:
        if self.jagged_edge_timing.check:
            int_array = array.astype(np.uint8)

            missing_point_filler_mask = cv.filter2D(int_array, -1, self.missing_point_filler_kernel) >= 3
            array[missing_point_filler_mask] = True

        self.jagged_edge_timing.increase()

