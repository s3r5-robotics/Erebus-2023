from mapping.mixed_grid import MixedGrid
import numpy as np


class UntraversableMapper:
    def __init__(self, grid: MixedGrid) -> None:
        self.__grid = grid

    def map_occupied(self):
        self.__grid.arrays["occupied"] = np.bitwise_or(self.__grid.arrays["walls"], self.__grid.arrays["holes"])

        self.__grid.arrays["occupied"][self.__grid.arrays["traversed"]] = False