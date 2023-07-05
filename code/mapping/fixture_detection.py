import copy
from typing import List

import cv2 as cv
import numpy as np
import skimage

from fixture_detection.color_filter import ColorFilter
from data_structures.angle import Angle
from data_structures.compound_pixel_grid import CompoundExpandablePixelGrid
from data_structures.vectors import Position2D, Vector2D
from robot.devices.camera import CameraImage


class FixtureDetector:
    def __init__(self, pixel_grid: CompoundExpandablePixelGrid) -> None:
        self.pixel_grid = pixel_grid

        # Color filtering
        self.colors = ("black", "white", "yellow", "red")
        self.color_filters = {
            "black": ColorFilter(lower=(0, 0, 0), upper=(30, 30, 30)),
            "white": ColorFilter(lower=(150, 150, 150), upper=(255, 255, 255)),
            "yellow": ColorFilter(lower=(155, 105, 0), upper=(200, 185, 90)),
            "red": ColorFilter(lower=(105, 0, 35), upper=(205, 175, 170))
        }

        self.max_detection_distance = 0.12 * 5

    def get_fixture_positions_and_angles(self, robot_position: Position2D, camera_image: CameraImage) -> list:
        positions_in_image = self.get_fixture_positions_in_image(np.flip(camera_image.image, axis=1))

        # debug = self.pixel_grid.get_colored_grid()

        fixture_positions = []
        fixture_angles = []
        for position in positions_in_image:
            relative_horizontal_angle = Angle(
                position[1] * (camera_image.data.horizontal_fov.radians / camera_image.data.width))

            fixture_horizontal_angle = (
                                               relative_horizontal_angle - camera_image.data.horizontal_fov / 2) + camera_image.data.horizontal_orientation

            fixture_horizontal_angle.normalize()

            camera_vector = Vector2D(camera_image.data.horizontal_orientation, camera_image.data.distance_from_center)
            camera_pos = camera_vector.to_position()
            camera_pos += robot_position

            detection_vector = Vector2D(fixture_horizontal_angle, self.max_detection_distance)
            detection_pos = detection_vector.to_position()

            detection_pos += camera_pos

            camera_array_index = self.pixel_grid.coordinates_to_array_index(camera_pos)
            detection_array_index = self.pixel_grid.coordinates_to_array_index(detection_pos)

            line_xx, line_yy = skimage.draw.line(camera_array_index[0], camera_array_index[1], detection_array_index[0],
                                                 detection_array_index[1])

            index = 0
            for x, y in zip(line_xx, line_yy):
                if x >= 0 and y >= 0 and x < self.pixel_grid.array_shape[0] and y < self.pixel_grid.array_shape[1]:
                    # debug[x, y] = (0, 255, 0)
                    back_index = index - 2
                    back_index = max(back_index, 0)
                    if self.pixel_grid.arrays["walls"][x, y]:
                        x1 = line_xx[back_index]
                        y1 = line_yy[back_index]
                        fixture_positions.append(self.pixel_grid.array_index_to_coordinates(np.array([x1, y1])))
                        fixture_angles.append(copy.deepcopy(fixture_horizontal_angle))
                        break
                index += 1

        # cv.imshow("fixture_detection_debug", debug)

        return fixture_positions, fixture_angles

    def get_fixture_positions_in_image(self, image: np.ndarray) -> List[Position2D]:
        image_sum = np.zeros(image.shape[:2], dtype=np.bool_)
        for filter in self.color_filters.values():
            image_sum += filter.filter(image) > 0

        image_sum = image_sum.astype(np.uint8) * 255

        contours, _ = cv.findContours(image_sum, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

        final_victims = []
        for c in contours:
            x, y, w, h = cv.boundingRect(c)
            final_victims.append(Position2D((x + x + w) / 2, (y + y + h) / 2))

        return final_victims

    def map_fixtures(self, camera_images, robot_position):
        for i in camera_images:
            positions, angles = self.get_fixture_positions_and_angles(robot_position, i)
            for pos, angle in zip(positions, angles):
                index = self.pixel_grid.coordinates_to_array_index(pos)
                self.pixel_grid.arrays["victims"][index[0], index[1]] = True
                self.pixel_grid.arrays["victim_angles"][index[0], index[1]] = angle.radians

    def mark_reported_fixture(self, robot_position, fixture_position):
        fixture_array_index = self.pixel_grid.coordinates_to_array_index(fixture_position)
        rr, cc = skimage.draw.disk(fixture_array_index, 4)
        self.pixel_grid.arrays["fixture_detection"][rr, cc] = True
