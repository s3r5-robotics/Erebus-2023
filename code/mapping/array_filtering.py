import cv2 as cv
import numpy as np
from flow_control.step_counter import StepCounter


class ArrayFilterer:
    def __init__(self) -> None:
        # Definiraj jedro za odstranjevanje izoliranih točk
        self.isolated_point_remover_kernel = np.array([[-2, -2, -2],
                                                       [-2, 1, -2],
                                                       [-2, -2, -2]])

        # Definiraj jedro za odstranjevanje neenakomernih robov
        self.jagged_edge_remover_kernel = np.array([[0, -1, 0],
                                                    [-1, 3, -1],
                                                    [0, -1, 0]])

        # Definiraj jedro za zapolnjevanje manjkajočih točk
        self.missing_point_filler_kernel = np.array([[0, 1, 0],
                                                     [1, 0, 1],
                                                     [0, 1, 0]])

        # Definiraj števec korakov za odstranjevanje izoliranih točk
        self.isolated_point_step_counter = StepCounter(100)

        # Definiraj števec korakov za glajenje robov
        self.jagged_edge_step_counter = StepCounter(100)

    def remove_isolated_points(self, array: np.ndarray) -> np.ndarray:
        # Preveri, ali je dovolj korakov opravljenih za odstranjevanje izoliranih točk
        if self.isolated_point_step_counter.check():
            # Ustvari masko izoliranih točk
            isolated_points_mask = cv.filter2D(array.astype(np.uint8), -1, self.isolated_point_remover_kernel) > 0
            # Odstrani izolirane točke iz polja
            array[isolated_points_mask] = False
        # Povečaj števec korakov
        self.isolated_point_step_counter.increase()

    def smooth_edges(self, array: np.ndarray) -> np.ndarray:
        # Preveri, ali je dovolj korakov opravljenih za glajenje robov
        if self.jagged_edge_step_counter.check():
            # Pretvori polje v celoštevilsko polje
            int_array = array.astype(np.uint8)
            # Ustvari masko manjkajočih točk
            missing_point_filler_mask = cv.filter2D(int_array, -1, self.missing_point_filler_kernel) >= 3
            # Zapolni manjkajoče točke v polju
            array[missing_point_filler_mask] = True

        # Povečaj števec korakov
        self.jagged_edge_step_counter.increase()
