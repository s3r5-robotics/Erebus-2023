import enum
from typing import Tuple, Optional

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

        # Limit PID output to maximum allowed motor velocity
        max_velocity = min(self._ml.max_velocity, self._mr.max_velocity)

        # TODO: Tune PID coefficients
        self._pid_rotating = PID(self._dt, (-max_velocity, max_velocity), kp=0.5, ki=0.1, kd=0.1)
        self._pid_moving = PID(self._dt, (-max_velocity, max_velocity), kp=0.5, ki=0.1, kd=0.1)

    def update(self) -> None:
        """Update internal states - shall be called once every timestep"""
        if debug.MOVEMENT:
            yaw, pitch, roll = self._imu.yaw_pitch_roll_dg
            print(f"Y|P|R  {yaw:.3f} | {pitch:.3f} | {roll:.3f}", end="    ")
            print(self.state.name, end="    ")

        if self.state == State.IDLE:
            return

        elif self.state == State.MOVING:
            # TODO: Detect being stuck, do not just bluntly increase speed
            actual_speed = self._gps.speed
            new_speed = self._pid_moving(actual_speed, self.target_speed)
            self._ml.target_velocity = new_speed
            self._mr.target_velocity = new_speed

            if debug.MOVEMENT:
                print(f"as|ts|ns  {actual_speed:.3f} | {self.target_speed:.3f} | {new_speed:.3f}", end="    ")

        elif self.state == State.ROTATING:
            actual_rotation = self._imu.yaw_dg
            # TODO: Handle wrap-around (e.g. rotating 10 degrees from 355 to 5 degrees)
            new_speed = self._pid_rotating(actual_rotation, self.target_rotation)
            # If error is positive (rotated more clockwise), then decrease left and increase right wheel speed
            self._ml.target_velocity = -new_speed
            self._mr.target_velocity = +new_speed

            if debug.MOVEMENT:
                print(f"ar|tr|ns  {actual_rotation:.3f} | {self.target_speed:.3f} | {new_speed:.3f}", end="    ")

        else:
            raise ValueError(f"Unknown state {self.state}")

        if debug.MOVEMENT:
            print(f"ML|MR  {self._ml.target_velocity:.3f} | {self._ml.target_velocity:.3f}", end="    ")

    def forward(self, speed: float) -> None:
        """Move forward at given speed (m/s)"""
        # TODO: If currently rotating, then we should enqueue this command (wait to stop rotating first), or what?
        self.state = State.MOVING
        self.target_speed = speed
        # TODO: Reset PID only if we were not already moving?
        self._pid_moving.reset()


class PID:
    """A proportional-integral-derivative (PID) controller"""

    # PID implementation based on
    # http://en.wikipedia.org/wiki/PID_controller
    # http://brettbeauregard.com/blog/category/pid/ (very good explanation from the author of the Arduino PID library)
    # https://github.com/br3ttb/Arduino-PID-Library
    # https://www.reddit.com/r/Python/comments/1qxp5d/pid_tuning_library/

    def __init__(self, time_step: float, output_range: Tuple[float, float], kp: float, ki: float, kd: float,
                 d_on_measurement: bool = True, p_on_measurement: bool = False) -> None:
        """
        Initialize P, PI, PD or PID controller with given coefficients and output range.

        :param time_step:        Sample period in seconds.
        :param output_range:     Output range for the controller (inclusive minimum, maximum values).
        :param kp:               Proportional coefficient for P, PI, PD and PID controllers.
        :param ki:               Integral coefficient for PI and PID controllers.
        :param kd:               Derivative coefficient for PD and PID controllers.
        :param d_on_measurement: If True, then the derivative term is calculated on the input directly instead
                                 of on the error (which is the classic way). Read more on:
                                 http://brettbeauregard.com/blog/2011/04/improving-the-beginner%e2%80%99s-pid-derivative-kick/
        :param p_on_measurement: If True, then the proportional term is calculated on the input directly instead of
                                 on the error (which is the classic way). Using proportional on measurement avoids
                                 overshoot for some types of systems.
                                 http://brettbeauregard.com/blog/2017/06/introducing-proportional-on-measurement/
                                 http://brettbeauregard.com/blog/2017/06/proportional-on-measurement-the-code/
        """
        assert kp is not None, "Proportional coefficient (kp) is mandatory"
        assert output_range[0] < output_range[1], "Output range must be a tuple (min, max) with min < max"

        # Parameters
        self._out_min = output_range[0]
        self._out_max = output_range[1]
        self._kp = kp
        self._ki = ki * time_step
        self._kd = kd / time_step
        self._p_on_measurement = p_on_measurement
        self._d_on_measurement = d_on_measurement

        # Internal state
        self._p = 0  # Proportional (saved state is required only if p_on_measurement)
        self._i = 0  # Integral (saved state is required by definition)

        self._input: Optional[float] = None  # Last input value (for derivative calculation if d_on_measurement)
        self._error: Optional[float] = None  # Last error value (for derivative calculation if not d_on_measurement)

    def _limit(self, value: float) -> float:
        return min(self._out_max, max(self._out_min, value))

    def __call__(self, measured_value: float, target_value: float) -> float:
        """
        Update the PID controller with new measurement

        :return: The calculated control output in the specified range.
        """
        # Calculate error terms
        error = target_value - measured_value
        d_input = (measured_value - self._input) if (self._input is not None) else 0
        d_error = (error - self._error) if (self._error is not None) else 0

        # Store variables for next calculation
        self._input = measured_value
        self._error = error

        # Calculate the proportional term (always present)
        self._p = (self._p - self._kp * d_input) if self._p_on_measurement else (self._kp * error)

        # Calculate the integral term in case of PI or PID controller
        self._i += self._ki * error
        # When PID output is at its maximum (saturated) but error is still present, integral term can be
        # accumulated to a very large value, preventing PID reaction if error is finally gone - this is
        # called integral windup. To avoid this, we limit the integral term to the output range.
        self._i = self._limit(self._i)

        # Calculate the derivative term in case of PD or PID controller
        d = -(self._kd * d_input) if self._d_on_measurement else (self._kd * d_error)

        # Compute PID Output
        return self._limit(self._p + self._i + d)

    def reset(self) -> None:
        """Reset internal state"""
        self._p = 0
        self._i = 0

        self._input = None
        self._error = None
