import random
import time
from pathlib import Path
from typing import Union

import cv2 as cv
import numpy as np
from PIL import Image
from numpy import ndarray

from controller import Robot, Motor, Camera

robot = Robot()
timestep = int(robot.getBasicTimeStep())

wheel_left: Motor = robot.getDevice("wheel_left motor")
wheel_right: Motor = robot.getDevice("wheel_right motor")
wheel_left.setPosition(float('inf'))
wheel_right.setPosition(float('inf'))

camera_left: Camera = robot.getDevice("camera_left")
camera_right: Camera = robot.getDevice("camera_right")
camera_left.enable(timestep)
camera_right.enable(timestep)


class ColorFilter:
    def __init__(self, lower: tuple[int, int, int], upper: tuple[int, int, int]):
        self.lower = np.array(lower)
        self.upper = np.array(upper)
        # self.lower = np.array(lower_hsv)
        # self.upper = np.array(upper_hsv)

    def filter(self, img):
        mask = cv.inRange(img, self.lower, self.upper)
        return mask


# Most important colors to avoid are:  Sunlit wall (rgb(59, 123, 134)), Shadow wall (rgb(18, 30, 32))
color_filters = {
    "black": ColorFilter(lower=(0, 0, 0), upper=(30, 30, 30)),
    "white": ColorFilter(lower=(150, 150, 150), upper=(255, 255, 255)),
    "yellow": ColorFilter(lower=(155, 105, 0), upper=(200, 185, 90)),
    "red": ColorFilter(lower=(105, 0, 35), upper=(205, 175, 170))
}


def camera_image(camera: Camera, rotate: int = None) -> np.ndarray:
    """
    The image is coded as a sequence of four bytes representing the `blue, green, red and alpha` levels of a pixel.
    Get the image from the camera and convert bytes to Width x Height x BGRA numpy array, then
    remove the unused alpha channel from each pixel to get WxHxBGR (40, 64, 3) numpy array.
    """
    arr = np.delete(np.frombuffer(camera.getImage(), np.uint8).reshape((40, 64, 4)), 3, axis=2)
    arr = np.flip(arr, axis=2)  # Swap blue and red channels to get RGB from BGR
    if rotate:  # Rotate image if needed
        arr = np.rot90(arr, k=rotate // 90)
    return arr  # .copy()  # Copy to avoid "ValueError: ndarray is not C-contiguous in cython"


def detect_color(image: ndarray, img_size: tuple[int, int] = (64, 40), cropped_img_size: tuple[int, int] = (54, 4),
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
    for f_name, f in color_filters.items():
        mask = f.filter(image)
        mask_size = cv.countNonZero(mask)
        # Check if the mask is larger than the minimum size and smaller than the entire image
        if max_mask_size >= mask_size >= min_mask_size:
            detected_colors.append(f_name)
            # save_img(mask, step_counter, mode="L")
    return detected_colors


def save_img(image: Union[Image, ndarray], name: Union[str, int] = None, mode: str = "RGB",
             image_dir: Path = None) -> None:
    """
    Save an image to `{image_dir}/{world_name}/{name}.png`.

    :param image: The image to save. Can be a PIL image or a numpy array.
    :param name: The name of the image (without the extension). If `None`, a random int value is used.
    :param mode: The mode to save the image in. Defaults to `RGB`.
    :param image_dir: The directory to save the image to. If `None`, the default directory is used:
    `/images`
    """
    if not name:
        name = random.randint(0, 9999)  # Prevent images from overriding each other

    if not image_dir:
        # If `image_dir` was not specified, use the default directory.
        image_dir = Path(Path.cwd() / "images")
        image_dir = image_dir.joinpath(Path(robot.world_path).stem)
        image_dir.mkdir(parents=True, exist_ok=True)

    # If the image is a numpy array, convert it to a PIL image
    if isinstance(image, np.ndarray):
        image = Image.fromarray(image, mode)

    im_path = image_dir.joinpath(f"{name}.png")
    image.save(im_path)
    print(f"Saved {name}:", im_path)


def main():
    step_counter: int = 0
    image_dir = Path(Path.cwd() / "images")
    image_dir = image_dir.joinpath(Path(robot.world_path).stem)
    image_dir.mkdir(parents=True, exist_ok=True)
    print("Saving images to:", image_dir)

    while robot.step(timestep) != -1:
        step_counter += 1

        # Every {x} time steps, randomly set wheel speeds
        if step_counter % 60 == 0:
            wheel_left.setVelocity(random.uniform(-6.28, 6.28))
            wheel_right.setVelocity(random.uniform(-6.28, 6.28))

        interval: int = 1  # The interval at which the robot should check for hazards
        # Every {x} time steps, check if there are
        if step_counter % interval == 0:
            for camera in [camera_left, camera_right]:
                image = camera_image(camera)
                # detected = detect_color(image)
                if True:  # if len(detected):
                    save_img(image, f"{step_counter}-{time.time()}")


if __name__ == "__main__":
    main()
