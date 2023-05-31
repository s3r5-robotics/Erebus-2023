import enum

import debug
from devices import GPS, InertialUnit, Motor


class State(enum.Enum):
    """Robot state"""
    IDLE = enum.auto()
    MOVING = enum.auto()
    ROTATING = enum.auto()


class Drivetrain:
    def __init__(self, left_motor: Motor, right_motor: Motor, imu: InertialUnit, gps: GPS) -> None:
        self._ml = left_motor
        self._mr = right_motor
        self._imu = imu
        self._gps = gps

        assert self._ml.time_step == self._mr.time_step == self._imu.time_step == self._gps.time_step
        self._dt = self._ml.time_step

        self.state = State.IDLE
        self.target_rotation = 0
        self.target_speed = 0

    def update(self) -> None:
        """Update internal states - shall be called once every timestep"""
        if debug.MOVEMENT:
            yaw, pitch, roll = self._imu.yaw_pitch_roll
            print(f"Y|P|R  {yaw.deg:.3f} | {pitch.deg:.3f} | {roll.deg:.3f}", end="    ")
            print(self.state.name, end="    ")

        if self.state == State.IDLE:
            return

        elif self.state == State.MOVING:
            actual_speed = self._gps.speed
            speed_error = actual_speed - self.target_speed
            if abs(speed_error) > 0.01:
                speed_fix = -0.1 * speed_error  # If going too fast (error positive), decrease the speed
                self._ml.target_velocity += speed_fix
                self._mr.target_velocity += speed_fix
            else:
                speed_fix = 0

            if debug.MOVEMENT:
                print(f"as|ts|fix  {actual_speed:.3f} | {self.target_speed:.3f} | {speed_fix:.3f}", end="    ")

        elif self.state == State.ROTATING:
            actual_rotation = self._imu.yaw
            rotation_error = actual_rotation.rotation_to(self.target_rotation)
            if abs(rotation_error.deg) > 1:
                speed_fix = 0.5 * rotation_error.deg
                # If error is positive (rotated more clockwise), then decrease left and increase right wheel speed
                self._ml.target_velocity -= speed_fix
                self._mr.target_velocity += speed_fix
            else:
                speed_fix = 0

            if debug.MOVEMENT:
                print(f"ar|tr|fix  {actual_rotation.deg:.3f} | {self.target_speed:.3f} | {speed_fix:.3f}", end="    ")

        if debug.MOVEMENT:
            print(f"ML|MR  {self._ml.target_velocity:.3f} | {self._ml.target_velocity:.3f}", end="    ")

    def forward(self, speed: float) -> None:
        """Move forward at given speed (m/s)"""
        # TODO: If currently rotating, then we should enqueue this command (wait to stop rotating first), or what?
        self.state = State.MOVING
        self.target_speed = speed
