import random
from pathlib import Path

import cv2 as cv
import numpy as np
import numpy.typing as npt
from PIL import Image

from fixture_detection.color_filter import ColorFilter


class FixtureDetector:
    def __init__(self) -> None:
        # Color filtering
        self.color_filters = {
            "black": ColorFilter(lower=(0, 0, 0), upper=(30, 30, 30)),
            "white": ColorFilter(lower=(150, 150, 150), upper=(255, 255, 255)),
            "yellow": ColorFilter(lower=(155, 105, 0), upper=(200, 185, 90)),
            "red": ColorFilter(lower=(105, 0, 35), upper=(205, 175, 170))
        }

    def detect_color(self, image: npt.ArrayLike, img_size: tuple[int, int] = (64, 40),
                     cropped_img_size: tuple[int, int] = (44, 4),
                     min_mask_size: int = 25, max_mask_size: int = 40) -> list[str]:
        """
        Vertically crop the camera's image down to the center `cropped_img_height` pixels.
        Check if the cropped image contains any colors from the color filters.

        :param image: The full image from the camera.
        :param img_size: The size of the original image
        :param cropped_img_size: The size of the resulting image after cropping
        :param min_mask_size: The minimum number of masked pixels.
        :param max_mask_size: `cropped_img_size[0] * cropped_img_size[1] - max_mask_size`.
        :return: A list of the detected colors' names.
        """
        if not max_mask_size:
            max_mask_size = cropped_img_size[0] * cropped_img_size[1]

        image = cv.cvtColor(image, cv.COLOR_BGRA2RGBA)
        og_image = image


        # Crop the image to the center `cropped_img_width[0]` pixels
        image = image[:, img_size[0] // 2 - (cropped_img_size[0] // 2):img_size[0] // 2 + (cropped_img_size[0] // 2)]
        # Crop the image to the center `cropped_img_height[1]` pixels
        image = image[img_size[1] // 2 - (cropped_img_size[1] // 2):img_size[1] // 2 + (cropped_img_size[1] // 2), :]

        detected_colors: list[str] = []
        for f_name, f in self.color_filters.items():
            mask = f.filter(image)
            mask_size = cv.countNonZero(mask)
            # Check if the mask is larger than the minimum size and smaller than the entire image
            if max_mask_size >= mask_size >= min_mask_size:
                detected_colors.append(f_name)
                # save_img(mask, step_counter, mode="L")

        if len(detected_colors) > 0:
            name = random.randint(0, 9999)  # Prevent images from overriding each other
            FixtureDetector.save_img(og_image, f"{name}_ogImage", mode="RGBA")
            FixtureDetector.save_img(mask, f"{name}_mask", mode="L")
        return detected_colors

    @staticmethod
    def save_img(image, name=None, mode: str = None,
                 image_dir=None) -> None:
        """
        Save an image to `{image_dir}/{world_name}/{name}.png`.

        :param image: The image to save. Can be a PIL image or a numpy array.
        :param name: The name of the image (without the extension). If `None`, a random int value is used.
        :param mode: The mode to save the image in. Defaults to `RGB`.
        :param image_dir: The directory to save the image to. If `None`, the default directory is used:
        `/images`
        """

        if not image_dir:
            # If `image_dir` was not specified, use the default directory.
            image_dir = Path(Path.cwd() / "images")
            image_dir.mkdir(parents=True, exist_ok=True)

        # If the image is a numpy array, convert it to a PIL image
        if isinstance(image, np.ndarray):
            image = Image.fromarray(image, mode)

        im_path = image_dir.joinpath(f"{name}.png")
        image.save(im_path)
        print(f"Saved {name}:", im_path)
