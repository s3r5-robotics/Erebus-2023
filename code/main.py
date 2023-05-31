import debug
from flow.state_machine import StateMachine
from robot import Robot
from utils import Angle

robot = Robot()
states = StateMachine("init")


def init(state):
    states.change_state("drive")


def drive(state):
    robot.drive.velocity = 1

    if robot.step_counter % 50 == 0:
        states.change_state("turn")


def turn(state):
    robot.drive.rotation = robot.drive.rotation + Angle(deg=90)
    states.change_state("drive")


states.create_state("init", init, {"drive"})
states.create_state("drive", drive, {"turn"})
states.create_state("turn", turn, {"drive"})

while robot():
    if debug.ANY:
        print(f"{robot.step_counter}", end="    ")

    if debug.DISTANCE:
        print(f"L|F|R  {robot.distance_l.value:.3f} | {robot.distance_f.value:.3f} | {robot.distance_r.value:.3f}",
              end="    ")

    states.run()

    if debug.ANY:
        print(flush=True)
