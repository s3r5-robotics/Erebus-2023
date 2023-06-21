import random
import time
from os import PathLike
from pathlib import Path
from typing import Literal

import cv2 as cv
import numpy as np
from numpy import ndarray

import controller
from controller import Robot, Motor, Camera
from image_plotter import ImagePlotter

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

plotter = ImagePlotter(width=64, height=40, columns=2)


class ColorFilter:
    def __init__(self, lower: tuple[int, int, int], upper: tuple[int, int, int]):
        self.lower = np.array(lower)
        self.upper = np.array(upper)
        # self.lower = np.array(lower_hsv)
        # self.upper = np.array(upper_hsv)

    def filter(self, img):
        hsv_image = cv.cvtColor(img, cv.COLOR_BGR2RGB)
        mask = cv.inRange(hsv_image, self.lower, self.upper)
        return mask


# RGB color lower and upper bounds
color_filters = {
    "black": ColorFilter(lower=(0, 0, 0), upper=(20, 20, 20)),
    "white": ColorFilter(lower=(200, 200, 200), upper=(255, 255, 255)),
    "yellow": ColorFilter(lower=(255, 145, 0), upper=(204, 255, 0)),
    "red": ColorFilter(lower=(196, 0, 0), upper=(255, 68, 0))
}

# color_filters = {
#     "black": ColorFilter(lower_hsv=(0, 0, 0), upper_hsv=(10, 10, 25)),
#     "white": ColorFilter(lower_hsv=(0, 0, 20), upper_hsv=(5, 5, 255)),
#     "yellow": ColorFilter(lower_hsv=(25, 140, 20), upper_hsv=(40, 255, 255)),
#     "red": ColorFilter(lower_hsv=(160, 150, 20), upper_hsv=(180, 255, 255))
# }

color_filter_palette = {  # Color for pixel value <=128, color for pixel value >128
    "black": ([255, 255, 255], [0, 0, 0]),
    "white": ([0, 0, 0], [255, 255, 255]),
    "yellow": ([0, 0, 0], [255, 255, 0]),
    "red": ([0, 0, 0], [255, 0, 0]),
}

min_fixture_height = 15
min_fixture_width = 15


def sum_images(images) -> bytes:
    """
    Combine the binary images into one.
    """
    final_img = images[0]
    for index, image in enumerate(images):
        final_img += image
    final_img[final_img > 255] = 255
    return final_img


def filter_fixtures(victims) -> list:
    """
    Filters out fixtures that are too small - far away.
    """
    final_victims = []
    for vic in victims:
        if vic["image"].shape[0] > min_fixture_height and vic["image"].shape[1] > min_fixture_width:
            final_victims.append(vic)

    return final_victims


def find_fixtures(image: np.ndarray, camera: int) -> list:
    """
    Finds fixtures in the image.
    Returns a list of dictionaries containing fixture positions and images.
    """
    binary_images: list[bytes] = []

    # Apply each color filter to the image
    for fn, f in color_filters.items():
        img = f.filter(image)
        binary_images.append(img)

        # Plot this image in the correct filter color
        img = plotter.add(img, mode="P", column=camera)[0]
        pixel0, pixel1 = color_filter_palette[fn]
        img.putpalette(128 * pixel0 + 128 * pixel1)

    binary_image: bytes = sum_images(binary_images)
    rect_image: ndarray = np.zeros_like(binary_image)

    contours, _ = cv.findContours(binary_image, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    for c0 in contours:
        x, y, w, h = cv.boundingRect(c0)
        cv.rectangle(binary_image, (x, y), (x + w, y + h), (225, 255, 255), -1)
        cv.rectangle(rect_image, (x, y), (x + w, y + h), (128, 128, 128), 1)
    contours, _ = cv.findContours(binary_image, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    final_victims = []
    for c in contours:
        x, y, w, h = cv.boundingRect(c)
        cv.rectangle(rect_image, (x, y), (x + w, y + h), (255, 255, 255), 1)
        final_victims.append({"image": image[y:y + h, x:x + w], "position": (x, y)})

    plotter.add(binary_image, mode="L", column=camera)
    plotter.add(rect_image, mode="L", column=camera)

    # print("unfiltered", len(final_victims))
    return filter_fixtures(final_victims)


images_dir = Path("C:/Programming/RoboCup_Erebus/FixtureDataset/C")  # Where the hazard images will be stored
debug_images_dir = Path(r"C:\Programming\RoboCup_Erebus\Erebus-2023\VictimDetection\images")

debug_images_dir = debug_images_dir.joinpath(Path(robot.world_path).stem)
debug_images_dir.mkdir(parents=True, exist_ok=True)


def camera_image(camera: controller.Camera, rotate: int = None) -> np.ndarray:
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


step_counter = 0
while robot.step(timestep) != -1:  # While the simulation is running
    step_counter += 1
    # Get the images from the cameras and convert bytes to `Width x Height x BRGA` numpy array,
    # then remove the alpha channel from each pixel to get `WxHxBRG (40, 64, 3)` numpy arrays.
    images = tuple(camera_image(cam, rot) for cam, rot in ((camera_left, 0), (camera_right, 0)))

    plotter.clear()
    plotter.add(images)

    # Every {x} time steps, randomly set wheel speeds
    # if step_counter % 70 == 0:
    #     wheel_left.setVelocity(random.uniform(-6.28, 6.28))
    #     wheel_right.setVelocity(random.uniform(-6.28, 6.28))
    wheel_left.setVelocity(0)
    wheel_right.setVelocity(0)

    for i, img in enumerate(images):
        victims = find_fixtures(img, i)  # Find all fixtures in the image
        if len(victims):
            print(step_counter, "victim")
            cv.imwrite(f"{images_dir}/{step_counter}-{time.time()}.png", img)

    if step_counter % 10 == 0:
        plotter.save(debug_images_dir.joinpath(f"{step_counter}.png"))
        print(step_counter, "image")
        break
