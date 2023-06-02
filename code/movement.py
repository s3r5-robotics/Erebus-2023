from typing import Tuple, Optional

import debug
from devices import GPS, InertialUnit, Motor
from utils import Angle


class Drivetrain:
    def __init__(self, left_motor: Motor, right_motor: Motor, imu: InertialUnit, gps: GPS) -> None:
        self._ml = left_motor
        self._mr = right_motor
        self._imu = imu
        self._gps = gps

        assert self._ml.time_step == self._mr.time_step == self._imu.time_step == self._gps.time_step
        self._dt = self._ml.time_step

        self.target_rotation: Optional[Angle] = None
        self.target_velocity = 0

        # Limit PID output to maximum allowed motor velocity
        self._max_motor_velocity = min(self._ml.max_velocity, self._mr.max_velocity)
        pid_range = (-self._max_motor_velocity, self._max_motor_velocity)
        # TODO: Tune PID coefficients
        self._pid_rotating = PID(self._dt, pid_range, kp=0.5, ki=0.1, kd=0.1)
        self._pid_moving = PID(self._dt, pid_range, kp=10, ki=2, kd=1)

    def __call__(self) -> None:
        """Update internal states - shall be called once every timestep"""

        velocity = self.velocity
        rotation = self.rotation

        if self.target_rotation is None:  # Fix for the first call (rotation cannot be retrieved in __init__)
            self.target_rotation = rotation

        # Calculater motor angular velocities (rotating speed)
        mv_moving = Angle(deg=180)  # self._pid_moving(velocity, self.target_velocity)
        mv_rotating = 0  # self._pid_rotating(rotation, self.target_rotation)

        # Calculate motor velocities
        # Because math angle convention is used, rotating in the positive angle direction means rotating
        # counter-clockwise - left motor rotates with negative velocity, right motor with positive.
        mv_l = mv_moving - mv_rotating
        mv_r = mv_moving + mv_rotating
        self.set_motor_velocity(mv_l, mv_r)

        # TODO: Detect being stuck, do not just bluntly increase speed

        if debug.MOVEMENT:
            yaw, pitch, roll = self._imu.yaw_pitch_roll
            print(f"Y|P|R  {yaw.deg:.3f} | {pitch.deg:.3f} | {roll.deg:.3f}", end="    ")
            print(f"av|tv|nms  {velocity:.6f} | {self.target_velocity:.3f} | {mv_moving:.3f}", end="    ")
            print(f"ar|tr|nrs  {rotation.deg:.3f} | {self.target_rotation.deg:.3f} | {mv_rotating:.3f}", end="    ")
            print(f"mm|mr  {mv_moving:.3f}/{Angle(mv_moving).deg:.3f} | {mv_rotating:.3f}/{Angle(mv_rotating).deg:.3f}"
                  f"  ml|mr  {mv_l:.3f}/{Angle(mv_l).deg:.3f} | {mv_r:.3f}/{Angle(mv_r).deg:.3f}", end="    ")

    # region Low-level control

    def _clamp_motor_velocity(self, velocity: float) -> float:
        return min(max(velocity, -self._max_motor_velocity), self._max_motor_velocity)

    def set_motor_velocity(self, left: float, right: float) -> None:
        """Set target rotational velocity for each wheel (radians/s)"""
        self._ml.target_velocity = self._clamp_motor_velocity(left)
        self._mr.target_velocity = self._clamp_motor_velocity(right)

    @property
    def velocity(self) -> float:
        """Current velocity (m/s)"""
        return self._gps.speed

    @velocity.setter
    def velocity(self, velocity: float) -> None:
        """Set new target velocity (m/s)"""
        self.target_velocity = velocity

    @property
    def rotation(self) -> Angle:
        """Current rotation"""
        return self._imu.yaw

    @rotation.setter
    def rotation(self, rotation: Angle) -> None:
        """Set new target rotation angle"""
        self.target_rotation = rotation

    # endregion Low-level control

    def forward(self, speed: float) -> None:
        """Move forward at given speed (m/s)"""
        self.target_velocity = speed


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
