import random
import time
from pathlib import Path

import cv2 as cv
import numpy as np
from PIL import Image

from controller import Robot, Motor, Camera
from image_plotter import ImagePlotter

robot = Robot()
timestep = int(robot.getBasicTimeStep())

wheel_left: Motor = robot.getDevice("wheel1 motor")
wheel_right: Motor = robot.getDevice("wheel2 motor")
wheel_left.setPosition(float('inf'))
wheel_right.setPosition(float('inf'))

camera_front: Camera = robot.getDevice("camera1")
camera_right: Camera = robot.getDevice("camera2")
camera_left: Camera = robot.getDevice("camera3")

camera_front.enable(timestep)
camera_right.enable(timestep)
camera_left.enable(timestep)

plotter = ImagePlotter(width=40, height=64, columns=3)


class ColorFilter:
    def __init__(self, lower_hsv, upper_hsv):
        self.lower = np.array(lower_hsv)
        self.upper = np.array(upper_hsv)

    def filter(self, img):
        hsv_image = cv.cvtColor(img, cv.COLOR_BGR2HSV)
        mask = cv.inRange(hsv_image, self.lower, self.upper)
        return mask


color_filters = {
    "black": ColorFilter(lower_hsv=(0, 0, 0), upper_hsv=(0, 0, 25)),
    "white": ColorFilter(lower_hsv=(0, 0, 200), upper_hsv=(0, 0, 220)),
    "yellow": ColorFilter(lower_hsv=(25, 140, 82), upper_hsv=(40, 255, 255)),
    "red": ColorFilter(lower_hsv=(160, 150, 127), upper_hsv=(180, 255, 255))
}
color_filter_palette = {  # Color for pixel value <=128, color for pixel value >128
    "black": ([255, 255, 255], [0, 0, 0]),
    "white": ([0, 0, 0], [255, 255, 255]),
    "yellow": ([0, 0, 0], [255, 255, 0]),
    "red": ([0, 0, 0], [255, 0, 0]),
}

min_fixture_height = 15
min_fixture_width = 15


def sum_images(images):
    final_img = images[0]
    for index, image in enumerate(images):
        final_img += image
    final_img[final_img > 255] = 255
    return final_img


def filter_fixtures(victims) -> list:
    final_victims = []
    for vic in victims:
        if vic["image"].shape[0] > min_fixture_height and vic["image"].shape[1] > min_fixture_width:
            final_victims.append(vic)

    return final_victims


def find_fixtures(image: Image, camera: int) -> list:
    """
    Finds fixtures in the image.
    Returns a list of dictionaries containing fixture positions and images.
    """
    image = np.rot90(image, k=3)
    binary_images = []
    for fn, f in color_filters.items():
        img = f.filter(image)
        binary_images.append(img)

        # Plot this image in the correct filter color
        img = plotter.add(img, mode="P", column=camera)[0]
        pixel0, pixel1 = color_filter_palette[fn]
        img.putpalette(128 * pixel0 + 128 * pixel1)

    binary_image = sum_images(binary_images)
    rect_image = np.zeros_like(binary_image)

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


# def get_unknown_color_clusters(image_path, known_colors_dict):
#     # Load the image
#     image = cv2.imread(image_path)
#
#     # Initialize a mask for all known colors
#     mask_known = np.zeros(image.shape[:2], dtype=np.uint8)
#
#     # Loop over the known colors and add to the mask
#     for color_name, (lower, upper) in known_colors_dict.items():
#         lower_known = np.array(lower)
#         upper_known = np.array(upper)
#         mask_color = cv2.inRange(image, lower_known, upper_known)
#         mask_known = cv2.bitwise_or(mask_known, mask_color)
#
#     # Invert the mask to get a mask of the unknown colors
#     mask_unknown = cv2.bitwise_not(mask_known)
#
#     # Bitwise-AND mask and original image to get an image of the unknown colors
#     unknown_colors_image = cv2.bitwise_and(image, image, mask=mask_unknown)
#
#     # Convert the image of unknown colors to grayscale
#     unknown_colors_gray = cv2.cvtColor(unknown_colors_image, cv2.COLOR_BGR2GRAY)
#     cv2.imshow("unknown_colors_gray", unknown_colors_gray)
#     cv2.waitKey(0)
#
#     # Threshold the grayscale image to get a binary image
#     _, binary_image = cv2.threshold(unknown_colors_gray, 1, 255, cv2.THRESH_BINARY)
#
#     # Find contours in the binary image
#     contours, _ = cv2.findContours(binary_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#
#     # Compute the area (or size) of each contour (or cluster)
#     cluster_sizes = [cv2.contourArea(c) for c in contours]
#
#     return cluster_sizes
#
#
# image_path = 'path_to_your_image.jpg'
# known_colors_dict = {
#     'wall_bright': ([194, 37, 45], [189, 59, 73]),
#     'wall_dark': ([189, 17, 11], [189, 55, 38]),
#     'floor_bright': ([0, 0, 83], [5, 7, 22]),
#     'floor_dark': ([0, 0, 28], [0, 8, 10])
# }


images_dir = "C:/Programming/RoboCup_Erebus/FixtureDataset/C"
debug_images_dir = r"C:\Programming\RoboCup_Erebus\Erebus-2023\VictimDetection\images"

debug_images_dir = Path(debug_images_dir, Path(robot.world_path).stem)
debug_images_dir.mkdir(parents=True, exist_ok=True)

step_counter = 0
while robot.step(timestep) != -1:
    step_counter += 1
    # Get the images from the cameras
    image_left = camera_left.getImage()
    image_front = camera_front.getImage()
    image_right = camera_right.getImage()
    images = (image_left, image_front, image_right)

    plotter.clear()
    plotter.add(images, mode="BRGA", rotations=[90, -90, -90])

    # Every {x} time steps, randomly set wheel speeds
    if step_counter % 70 == 0:
        wheel_left.setVelocity(random.uniform(-6.28, 6.28))
        wheel_right.setVelocity(random.uniform(-6.28, 6.28))

    for i, img in enumerate(images):
        numpy_image = np.array(np.frombuffer(img, np.uint8).reshape((40, 64, 4)))
        victims = find_fixtures(numpy_image, i)
        if len(victims):
            print(step_counter, "victim")
            cv.imwrite(f"{images_dir}/{step_counter}-{time.time()}.png", numpy_image)

    if step_counter % 10 == 0:
        plotter.save(debug_images_dir.joinpath(f"{step_counter}.png"))
        print(step_counter, "image")