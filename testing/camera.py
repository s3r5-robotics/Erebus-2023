from controller import Robot, Camera
import cv2
import numpy as np
import time

robot = Robot()

camera_right = robot.getDevice("camera_right")
camera_left = robot.getDevice("camera_left")

timestep = int(robot.getBasicTimeStep())

camera_right.enable(timestep)
camera_left.enable(timestep)

while robot.step(timestep) != -1:
    # Convert the camera_right image into an openCV Mat. You must pass in the height
    # and width of the camera_right in pixels, as well as specify CV_8UC4, meaning that
    # it is in RGBA format

    image_right = camera_right.getImage()
    image_right = np.frombuffer(image_right, np.uint8).reshape((camera_right.getHeight(), camera_right.getWidth(), 4))
    frame_right = cv2.cvtColor(image_right, cv2.COLOR_BGRA2BGR)

    image_left = camera_right.getImage()
    image_left = np.frombuffer(image_left, np.uint8).reshape((camera_left.getHeight(), camera_left.getWidth(), 4))
    frame_left = cv2.cvtColor(image_right, cv2.COLOR_BGRA2BGR)

    cv2.imshow("frame_right", frame_right)
    cv2.imshow("frame_left", frame_left)

    cv2.waitKey(1)
