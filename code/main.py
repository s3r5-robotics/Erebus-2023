import debug
from flow.state_machine import StateMachine, State
from robot import Robot
from utils import Angle

robot = Robot()
states = StateMachine("init")

TILE_SIZE = 120  # mm
MIN_DIST = 1.2 * (TILE_SIZE / 2 - 37)  # 37 is the offset of the distance sensor from the center of the robot


def init() -> bool:
    states.change_state("drive")
    return True


def drive():
    if not robot.drive.rotated:
        # Robot is still rotating
        return True

    if robot.distance_f.mm > MIN_DIST:
        # Robot can get stuck in some narrow corridors. It can happen that robot is not centered to the tile, but
        # if the corridor is wide enough, at least one of the distance sensors must have a distance greater than
        # minimal distance. If both are less, this is not a normal path.
        if robot.distance_l.mm < MIN_DIST and robot.distance_r.mm < MIN_DIST:
            states.change_state("turn")
        else:
            robot.drive.velocity = 1
    else:
        robot.drive.velocity = 0
        states.change_state("turn")

    return True


def turn():
    if robot.distance_l.mm > TILE_SIZE:
        robot.drive.rotation += Angle(deg=90)
    elif robot.distance_r.mm > TILE_SIZE:
        robot.drive.rotation -= Angle(deg=90)
    else:
        robot.drive.rotation += Angle(deg=180)

    states.change_state("drive")
    return True


def end_run():
    return False


states += State.for_function(init, drive, turn)
states += State.for_function(drive, turn, end_run)
states += State.for_function(turn, drive, end_run)
states += State.for_function(end_run)

while robot.step():
    if debug.ANY:
        print(f"{robot.step_counter}", end="    ")

    robot()

    if debug.DISTANCE:
        dl, df, dr = robot.distances
        print(f"L|F|R  {dl:.3f} | {df:.3f} | {dr:.3f}", end="    ")

    robot()

    if not states.run():
        print("State machine complete")
        break

    if debug.ANY:
        print(flush=True)
