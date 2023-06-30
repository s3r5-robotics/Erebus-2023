import debug
from fixtures import FixtureClassifier, FixtureDetector
from flow.state_machine import StateMachine, State
from robot import Robot
from utils import Angle

robot = Robot()
fixture_detector = FixtureDetector()
states = StateMachine("init")
fixture_classifier = FixtureClassifier("rev-1")

TILE_SIZE = 120  # mm
MIN_DIST = 1.2 * (TILE_SIZE / 2 - 37)  # 37 is the offset of the distance sensor from the center of the robot
HAZ_DETECTION_INTERVAL: int = 10  # The interval at which the robot should check the cameras' images for hazards


def init() -> bool:
    states.change_state("drive")
    return True


def drive():
    dl, df, dr = robot.distances

    if not robot.drive.rotated:
        # Robot is still rotating
        return True

    if df > MIN_DIST:
        # Robot can get stuck in some narrow corridors. It can happen that robot is not centered to the tile, but
        # if the corridor is wide enough, at least one of the distance sensors must have a distance greater than
        # minimal distance. If both are less, this is not a normal path.
        if dl < MIN_DIST and dr < MIN_DIST:
            states.change_state("turn")
        else:
            robot.drive.velocity = 1
    else:
        robot.drive.velocity = 0
        states.change_state("turn")

    return True


def turn():
    dl, df, dr = robot.distances
    if dl > TILE_SIZE:
        robot.drive.rotation += Angle(deg=90)
    elif dr > TILE_SIZE:
        robot.drive.rotation -= Angle(deg=90)
    else:
        robot.drive.rotation += Angle(deg=180)

    states.change_state("drive")
    return True


def end_run():
    return False


# Create states and possible states that it can change to
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

    if robot.step_counter % HAZ_DETECTION_INTERVAL == 0:
        for camera in [robot.camera_l, robot.camera_r]:
            image = fixture_detector.get_image(camera)
            detected = fixture_detector.detect_color(image)
            if len(detected):
                if debug.FIXTURE_DETECTION:
                    print(f"Detected fixture color: {detected}")

                # TODO: navigate closer to the fixture

                fixture: str = fixture_classifier.classify_fixture(image)
                print("Fixture:", fixture)

    robot()

    if not states.run():
        print("State machine complete")
        break

    if debug.ANY:
        print(flush=True)
