import random
from pathlib import Path
from typing import Literal

import cv2 as cv
import keras
import numpy as np
import tensorflow as tf
import numpy.typing as npt
from PIL import Image

import debug
from controller import Camera


class ColorFilter:
    def __init__(self, lower: tuple[int, int, int], upper: tuple[int, int, int]):
        self.lower = np.array(lower)
        self.upper = np.array(upper)

    def filter(self, img):
        mask = cv.inRange(img, self.lower, self.upper)
        return mask


class FixtureDetector:
    # Most important colors to avoid are:  Sunlit wall (rgb(59, 123, 134)), Shadow wall (rgb(18, 30, 32))
    color_filters = {
        "black": ColorFilter(lower=(0, 0, 0), upper=(30, 30, 30)),
        "white": ColorFilter(lower=(150, 150, 150), upper=(255, 255, 255)),
        "yellow": ColorFilter(lower=(155, 105, 0), upper=(200, 185, 90)),
        "red": ColorFilter(lower=(105, 0, 35), upper=(205, 175, 170))
    }

    @staticmethod
    def get_image(camera: Camera, rotate: int = None,
                  im_dimensions: tuple[int, int, int] = (40, 64, 4)) -> np.ndarray:
        """
        Take the `camera`'s image, return it as a numpy array as an RGB image of given size.

        The image is coded as a sequence of four bytes representing the `blue, green, red and alpha` levels of a pixel.
        Get the image from the camera and convert bytes to Width x Height x BGRA numpy array, then
        remove the unused alpha channel from each pixel to get WxHxBGR (40, 64, 3) numpy array.

        :param camera: The camera to get the image from.
        :param rotate: Rotate the image by `rotate` degrees clockwise.
        :param im_dimensions: The dimensions of the image to return; [height, width, *channels]
        *the camera from the image has 4 channels (RGBA), but only RGB is returned.
        """
        arr = np.delete(np.frombuffer(camera.getImage(), np.uint8).reshape(im_dimensions), 3, axis=2)
        arr = np.flip(arr, axis=2)  # Swap blue and red channels to get RGB from BGR
        if rotate:  # Rotate image if needed
            arr = np.rot90(arr, k=rotate // 90)
        return arr  # .copy()  # Copy to avoid "ValueError: ndarray is not C-contiguous in cython"

    @staticmethod
    def detect_color(image: npt.ArrayLike, img_size: tuple[int, int] = (64, 40),
                     cropped_img_size: tuple[int, int] = (44, 4),
                     min_mask_size: int = 25, max_mask_size: int = 40,
                     step_counter: int = None) -> list[str]:
        """
        Vertically crop the camera's image down to the center `cropped_img_height` pixels.
        Check if the cropped image contains any colors from the color filters.

        :param image: The full image from the camera.
        :param img_size: The size of the original image
        :param cropped_img_size: The size of the resulting image after cropping
        :param min_mask_size: The minimum number of masked pixels.
        :param max_mask_size: `cropped_img_size[0] * cropped_img_size[1] - max_mask_size`.
        :param step_counter: Used as the name of the saved image. If `None`, a random int value is used.
        :return: A list of the detected colors' names.
        """
        if not max_mask_size:
            max_mask_size = cropped_img_size[0] * cropped_img_size[1]

        # Crop the image to the center `cropped_img_width[0]` pixels
        image = image[:, img_size[0] // 2 - (cropped_img_size[0] // 2):img_size[0] // 2 + (cropped_img_size[0] // 2)]
        # Crop the image to the center `cropped_img_height[1]` pixels
        image = image[img_size[1] // 2 - (cropped_img_size[1] // 2):img_size[1] // 2 + (cropped_img_size[1] // 2), :]

        detected_colors: list[str] = []
        for f_name, f in FixtureDetector.color_filters.items():
            mask = f.filter(image)
            mask_size = cv.countNonZero(mask)
            # Check if the mask is larger than the minimum size and smaller than the entire image
            if max_mask_size >= mask_size >= min_mask_size:
                detected_colors.append(f_name)
                # save_img(mask, step_counter, mode="L")
        return detected_colors


class FixtureClassifier:
    CLASSES = ['0', 'C', 'F', 'H', 'O', 'P', 'S', 'U']
    MODEL_DATA = {
        "rev-1": "../test_fr-hp.keras",
        "rev-2": "../test_frt-hp.keras"
    }

    def __init__(self, model: Literal["rev-1", "rev-2"]):
        self.model = keras.models.load_model(self.MODEL_DATA[model])

    def classify_fixture(self, fixture: npt.ArrayLike) -> str:
        img_array = tf.expand_dims(fixture, 0)
        predictions = self.model.predict(img_array)
        predictions_sorted = sorted(map(lambda p, c: (c, p), predictions[0], self.CLASSES), key=lambda t: -t[1])
        prediction: tuple[str, float] = predictions_sorted[0]

        if debug.FIXTURE_FITTING:
            print(predictions_sorted)

        return prediction[0]


