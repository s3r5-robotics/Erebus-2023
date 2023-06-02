import debug
from flow.state_machine import StateMachine, State
from robot import Robot
from utils import Angle

robot = Robot()
states = StateMachine("init")


def init() -> bool:
    states.change_state("drive")
    return True


def drive():
    robot.drive.velocity = 1
    if robot.step_counter % 50 == 0:
        states.change_state("turn")
    return True


def turn():
    turn.num_turns = getattr(turn, "num_turns", 0) + 1

    robot.drive.rotation = robot.drive.rotation + Angle(deg=90)
    states.change_state("drive")

    if turn.num_turns > 4:
        states.change_state("end_run")

    return True


def end_run():
    return False


states += State.for_function(init, drive, turn)
states += State.for_function(drive, turn, end_run)
states += State.for_function(turn, drive, end_run)
states += State.for_function(end_run)

while robot():
    if debug.ANY:
        print(f"{robot.step_counter}", end="    ")

    if debug.DISTANCE:
        print(f"L|F|R  {robot.distance_l.value:.3f} | {robot.distance_f.value:.3f} | {robot.distance_r.value:.3f}",
              end="    ")

    if not states.run():
        print("State machine complete")
        break

    if debug.ANY:
        print(flush=True)
