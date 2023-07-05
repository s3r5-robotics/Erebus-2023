import cv2 as cv
import numpy as np


class ColorFilter:
    def __init__(self, lower: tuple[int, int, int], upper: tuple[int, int, int]):
        self.lower = np.array(lower)
        self.upper = np.array(upper)

    def filter(self, img):
        mask = cv.inRange(img, self.lower, self.upper)
        return mask
