from controller import Robot, Camera, Motor
import cv2
import numpy as np
import datetime


robot = Robot()
# camera_right = robot.getDevice("camera_right")
camera_left = robot.getDevice("camera_left")
camera_right = robot.getDevice("camera_right")
timestep = int(robot.getBasicTimeStep())

wheel_left = robot.getDevice("wheel1 motor")
wheel_right = robot.getDevice("wheel2 motor")
wheel_left.setPosition(float('inf'))
wheel_right.setPosition(float('inf'))

camera_left.enable(timestep)
# camera_right.enable(timestep)


# Available targets:
#   H, S, U
#   F, P, C, O
cur_target: str = "U"
im_location: str = r"C:\Programming\RoboCup_Erebus\Erebus v23\Erebus-2022 - Code\testing\images" + \
                   f"\\{cur_target}\\{cur_target}_"

i: int = 0
while robot.step(timestep) != -1:
    wheel_left.setVelocity(0)
    wheel_right.setVelocity(3.0)

    image_left = camera_left.getImage()
    image_left = np.frombuffer(image_left, np.uint8).reshape((camera_left.getHeight(), camera_left.getWidth(), 4))
    frame_left = cv2.cvtColor(image_left, cv2.COLOR_BGRA2BGR)

    # image_right = camera_right.getImage()
    # image_right = np.frombuffer(image_right, np.uint8).reshape((camera_left.getHeight(), camera_left.getWidth(), 4))
    # frame_right = cv2.cvtColor(image_right, cv2.COLOR_BGRA2BGR)

    if i % 2 == 0:
        now = datetime.datetime.now()
        print(f"Saving image {i}...")
        cv2.imwrite(im_location + now.strftime("%H%M%S-%f_l") + ".png", frame_left)
        # cv2.imwrite(im_location + now.strftime("%H%M%S-%f_r") + ".png", frame_right)

    i += 1


