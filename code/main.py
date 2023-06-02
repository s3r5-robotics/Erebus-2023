import debug
from robot import Robot
from utils import Angle

robot = Robot()

while robot():
    if debug.ANY:
        print(f"{robot.step_counter}", end="    ")

    if robot.step_counter == 1:
        robot.drive.velocity = 1
    elif robot.step_counter % 100 == 0:
        robot.drive.rotation = robot.drive.rotation + Angle(deg=90)

    if debug.DISTANCE:
        print(f"L|F|R  {robot.distance_l.value:.3f} | {robot.distance_f.value:.3f} | {robot.distance_r.value:.3f}",
              end="    ")

    if debug.ANY:
        print(flush=True)
