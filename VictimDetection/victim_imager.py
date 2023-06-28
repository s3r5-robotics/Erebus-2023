import random
import random
import time
from pathlib import Path
from typing import Union

import cv2 as cv
import numpy as np
from PIL import Image
from numpy import ndarray

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
        # hsv_image = cv.cvtColor(img, cv.COLOR_BGR2RGB)
        mask = cv.inRange(img, self.lower, self.upper)
        return mask


# RGB color lower and upper bounds
color_filters = {
    "black": ColorFilter(lower=(0, 0, 0), upper=(70, 70, 70)),
    "white": ColorFilter(lower=(80, 80, 80), upper=(255, 255, 255)),
    "yellow": ColorFilter(lower=(204, 126, 16), upper=(200, 255, 0)),
    "red": ColorFilter(lower=(107, 37, 37), upper=(204, 113, 16))
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

    contours, _ = cv.findContours(binary_image, cv.RETR_TREE, cv.CHAIN_APPROX_TC89_L1)
    final_victims = []
    for c in contours:
        x, y, w, h = cv.boundingRect(c)
        final_victims.append({"image": image[y:y + h, x:x + w], "position": (x, y)})

    plotter.add(binary_image, mode="L", column=camera)
    plotter.add(rect_image, mode="L", column=camera)

    # print("unfiltered", len(final_victims))
    return filter_fixtures(final_victims)


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


def main():
    # Where the hazard images will be stored. Change the last letter based on the hazard type.
    images_dir = Path(r"C:\Programming\RoboCup_Erebus\FixtureDataset\C")

    # Debug images are stored in the folder with the world name
    debug_images_dir = Path(r"C:\Programming\RoboCup_Erebus\Erebus-2023\VictimDetection\images")
    debug_images_dir = debug_images_dir.joinpath(Path(robot.world_path).stem)
    debug_images_dir.mkdir(parents=True, exist_ok=True)

    # List of cameras and their rotation (0 = horizontal)
    cameras_rotations: list[tuple[Camera, int]] = [(camera_left, 0),
                                                   (camera_right, 0)]

    step_counter = 0
    while robot.step(timestep) != -1:  # While the simulation is running
        step_counter += 1
        # Get the images from the cameras and convert bytes to `Width x Height x BRGA` numpy array,
        # then remove the alpha channel from each pixel to get `WxHxBRG (64, 40, 3)` numpy arrays.
        images = tuple(camera_image(cam, rot) for cam, rot in cameras_rotations)

        plotter.clear()
        plotter.add(images)

        # Every {x} time steps, randomly set wheel speeds
        # if step_counter % 70 == 0:
        #     wheel_left.setVelocity(random.uniform(-6.28, 6.28))
        #     wheel_right.setVelocity(random.uniform(-6.28, 6.28))
        wheel_left.setVelocity(0)
        wheel_right.setVelocity(0)

        # Save camera image if victim is detected
        for i, img in enumerate(images):
            victims = find_fixtures(img, i)  # Find all fixtures in the image
            if len(victims):
                print(step_counter, "victim")
                cv.imwrite(f"{images_dir}/{step_counter}-{time.time()}.png", img)

        # Generate and save a debug image separated into color filters
        if step_counter % 10 == 0:
            im_path = debug_images_dir.joinpath(f"{step_counter}.png")
            plotter.save(im_path)
            print(step_counter, "generated debug image:", im_path)
            break


def detect_color(image: ndarray, img_height: int = 40, cropped_img_height: int = 4):
    """
    Vertically crop the camera's image down to the center 4 pixels.
    Check if the cropped image contains any colors from the color filters.

    :return: List of colors detected.
    """
    im_seed = random.randint(000, 999)  # The name of the images

    save_img(image, f"{im_seed}-original")
    # Crop the image to the center `cropped_img_height` pixels
    image = image[img_height // 2 - (cropped_img_height // 2):img_height // 2 + (cropped_img_height // 2), :]
    save_img(image, f"{im_seed}-cropped")

    _color_filters = {
        "black": ColorFilter(lower=(0, 0, 0), upper=(30, 30, 30)),
        "white": ColorFilter(lower=(200, 200, 200), upper=(255, 255, 255)),
        "yellow": ColorFilter(lower=(155, 105, 0), upper=(200, 185, 90)),
        "red": ColorFilter(lower=(105, 0, 35), upper=(205, 175, 180))
    }
    detected_colors: list[str] = []
    final_mask = np.zeros(image.shape[:2], dtype=np.uint8)
    for filter_name, filter in _color_filters.items():
        mask = filter.filter(image)
        if cv.countNonZero(mask) > 0:
            print(f"Detected {filter_name}!")
            save_img(mask, f"{im_seed}-mask_{filter_name}", mode="L")
            detected_colors.append(filter_name)
            final_mask = cv.bitwise_or(final_mask, mask)
    save_img(final_mask, f"{im_seed}-finalMask", mode="L")
    return detected_colors


def save_img(image: Union[Image, ndarray], name: str, mode: str = "RGB", image_dir: Path = None) -> None:
    """
    Save an image to `{image_dir}/{world_name}/{name}.png`.

    :param image: The image to save. Can be a PIL image or a numpy array.
    :param name: The name of the image (without the extension).
    :param mode: The mode to save the image in. Defaults to `RGB`.
    :param image_dir: The directory to save the image to. If `None`, the default directory is used:
    `/images`
    """
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


if __name__ == "__main__":
    # main()
    stepcounter = 0
    while robot.step(timestep) != -1:
        stepcounter += 1
        wheel_left.setVelocity(0)
        wheel_right.setVelocity(0)
        if stepcounter < 25:
            continue
        images = camera_image(camera_right, 0)
        print(detect_color(images))
        break
