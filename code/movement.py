import collections
import enum
import math
import time
from typing import Tuple, Optional, Callable, NamedTuple, Deque

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
        self._max_motor_velocity = min(self._ml.max_velocity, self._mr.max_velocity)

        # TODO: Tune PID coefficients
        self._pid_rotating = PID(self._dt, (-self.max_motor_velocity, self.max_motor_velocity), kp=0.5, ki=0.1, kd=0.1)
        self._pid_moving = PID(self._dt, (-self.max_motor_velocity, self.max_motor_velocity), kp=0.5, ki=0.1, kd=0.1)

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

    @property
    def max_motor_velocity(self) -> float:
        """Maximum velocity of the motors in radians/s"""
        return self._max_motor_velocity

    def _clamp_motor_velocity(self, velocity: float) -> float:
        return min(max(velocity, -self.max_motor_velocity), self.max_motor_velocity)

    def set_motor_velocity(self, left: float, right: float) -> None:
        """Set target rotational velocity for each wheel (radians/s)"""
        self._ml.target_velocity = self._clamp_motor_velocity(left)
        self._mr.target_velocity = self._clamp_motor_velocity(right)


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

        :param measured_value: The measured, actual value.
        :param target_value:   The target value the system (measured_value) should achieve.

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


class PIDTunner:
    """
    Class to help determine optimal coefficients of the PID controller

    References:
    - Explore the 3 PID tuning methods
      https://www.incatools.com/pid-tuning/pid-tuning-methods/
    - Standard PID Tuning Methods
      https://pages.mtu.edu/~tbco/cm416/tuning_methods.pdf
    - Comparison Study of PID Controller Tuning using Classical/Analytical Methods
      https://www.ripublication.com/ijaer18/ijaerv13n8_07.pdf
    - Comparison of PID Controller Tuning Methods
      http://maulana.lecture.ub.ac.id/files/2014/12/Jurnal-PID_Tunning_Comparison.pdf
    """

    # noinspection SpellCheckingInspection
    class Method(enum.Enum):
        """Tuning methods with factors Kp, Ki, Kd"""
        ZIEGLER_NICHOLS = (34, 40, 160)
        """https://en.wikipedia.org/wiki/Ziegler%E2%80%93Nichols_method"""
        TYREUS_LUYBEN = (44, 9, 126)
        CIANCONE_MARLIN = (66, 88, 162)
        PESSEN_INTEGRAL = (28, 50, 133)
        SOME_OVERSHOOT = (60, 40, 60)
        NO_OVERSHOOT = (100, 40, 60)

    class State(enum.Enum):
        """Tuning states"""
        STEP_UP = enum.auto()
        STEP_DOWN = enum.auto()
        DONE = enum.auto()
        FAILED = enum.auto()

    class Peak(NamedTuple):
        timestamp: float
        value: float

    def __init__(self, time_step: float,
                 target_value: float, output_step: float, output_zero: float = 0, target_value_noise: float = None,
                 method: Method = Method.ZIEGLER_NICHOLS, lookback: float = 10,
                 max_peaks: int = 20, peak_amplitude_tolerance: float = 0.05,
                 time_function: Callable[[], float] = time.time) -> None:
        """
        :param time_step:          Sample period in seconds.
        :param target_value:       The target value the system (measured_value) should achieve.
        :param output_step:        The step size of the output value. Make sure that the system can react
                                   to this step size and reach the `target_value`, however take care that
                                   the `output_zero` +- `output_step` stays within the range the system
                                   can handle.
        :param output_zero:        The offset of the output value, e.g. if the system can only handle positive
                                   values or if the steady state of the system is not at output value 0.
        :param target_value_noise: This value specifies how much the measured value must reach above or below the
                                   target value to be considered as oscillation around the target value. This is
                                   useful if the measured value is noisy and naturally oscillates around the target
                                   value - in this case the oscillation is caused by the noise and not by the output
                                   value. This value shall therefore be set to a value high enough to be able to
                                   detect peaks in the oscillations, but low enough for the system to still be
                                   able to oscillate at least 5 times in `lookback` seconds by switching the output
                                   value between `output_zero` +- `output_step`.
                                   If None, it will be set to 10% of the target_value.
        :param method:             The tuning method to use.
        :param lookback:           The time in seconds to look back to check the peaks in the measured values.
        :param max_peaks:          The maximum number of peaks to check before failing if their amplitude does
                                   not converge to `peak_amplitude_tolerance`.
        :param peak_amplitude_tolerance: The tolerance of the target value when detecting if the measured value
                                         has reached the target value.
        :param time_function:      The function to use to get the current time.
        """
        self._get_time = time_function
        self._method = method  # TODO: Not all methods use the same relay method
        self._state = self.State.STEP_UP

        self._target_value = target_value
        self._target_value_noise = (target_value * 0.1) if (target_value_noise is None) else target_value_noise

        self._output_zero = output_zero
        self._output_step = output_step
        self._output = 0

        # numpy array could be used, however there are no package dependencies so far and the deque is efficient
        # enough for this purpose (relatively low number of samples and high performance is not the point).
        self._inputs: Deque[float] = collections.deque(maxlen=round(lookback / time_step))
        self._last_extrema = 0  # 0: none, 1: maximum, -1: minimum
        # Only last 4 peaks are required for amplitude difference calculation
        self._peaks: Deque[PIDTunner.Peak] = collections.deque(maxlen=4)
        self._peak_count = 0
        self._max_peaks = max_peaks
        self._peak_amplitude_tolerance = peak_amplitude_tolerance

        self._ku = 0  # Ultimate gain
        self._pu = 0  # Ultimate period

    def _log(self, message: str = None) -> None:
        print(f"[{type(self).__name__} {self._get_time():.3f} {self._state.name}]: {message}" if message else "")

    @property
    def done(self) -> bool:
        return self._state in (self.State.DONE, self.State.FAILED)

    @property
    def completed(self) -> bool:
        return self._state is self.State.DONE

    @property
    def failed(self) -> bool:
        return self._state is self.State.FAILED

    @property
    def parameters(self) -> Tuple[float, float, float]:
        """Get the calculated Kp, Ki and Kd parameters"""
        if not self.completed:
            raise RuntimeError(f"PID tuning is not complete yet ({self._state.name})")

        dp, di, dd = self._method.value
        kp = self._ku / dp
        ki = kp / (self._pu / di)
        kd = kp * (self._pu / dd)

        return kp, ki, kd

    def __call__(self, measured_value: float) -> float:
        """
        Perform one step of the PID tuning algorithm, based on the new measured value

        :param measured_value: The measured, actual value.

        :return: Output value which shall be set.
        """
        if self._state in (self._state.DONE, self._state.FAILED):
            self._log(f"PID tuning is already finished ({self._state.name})")
            return self._output_zero

        # Change state if input has reached the min or max target value
        target_value_max = self._target_value + self._target_value_noise
        target_value_min = self._target_value - self._target_value_noise
        if (self._state is self._state.STEP_UP) and measured_value > target_value_max:
            self._log(f"Reached max target value {measured_value} > {target_value_max}, STEP_DOWN")
            self._state = self._state.STEP_DOWN
        elif (self._state is self._state.STEP_DOWN) and measured_value < target_value_min:
            self._log(f"Reached min target value {measured_value} < {target_value_min}, STEP_UP")
            self._state = self._state.STEP_UP

        # Output is always set in "relay mode", meaning it is either -100% or 100%
        if self._state is self._state.STEP_UP:
            self._output = self._output_zero + self._output_step
        elif self._state is self._state.STEP_DOWN:
            self._output = self._output_zero - self._output_step

        # Check if the peak value has been reached - before adding the current value, otherwise both
        # would always result in True because of the <= and >= comparison).
        is_max = measured_value >= max(self._inputs) if self._inputs else False
        is_min = measured_value <= min(self._inputs) if self._inputs else False

        # Store the measured value for peak checking
        self._inputs.append(measured_value)
        self._log(f"input={measured_value:.3f} ({target_value_min:.3f}..{target_value_max:.3f}),"
                  f" output={self._output:.3f}, inputs={len(self._inputs)}/{self._inputs.maxlen},"
                  f" is_max={is_max}, is_min={is_min}")

        # Wait enough samples to be collected, otherwise the peak detection is incorrect (local maxima/minima)
        if len(self._inputs) < self._inputs.maxlen:
            return self._output

        # Not every maximum is a peak - curve can rise, stabilise, rise again, stabilise, ... , then fall.
        # So only take this maximum into account if the previous peak was a minimum (and vice versa) to
        # detect the peaks of the oscillating signal (which the input value shall be in the "relay" method).
        # This is called the inflection point - the location where a curve actually changes the slope direction.
        is_peak = False
        # noinspection PyUnboundLocalVariable
        if is_max:
            if self._last_extrema == -1:
                is_peak = True
            self._last_extrema = 1
        elif is_min:
            if self._last_extrema == 1:
                is_peak = True
            self._last_extrema = -1

        if is_peak:
            self._peaks.append(self.Peak(self._get_time(), measured_value))
            self._peak_count += 1

        self._log(f"is_max={is_max}, is_min={is_min}, is_peak={is_peak}, last_extrema={self._last_extrema},"
                  f"peak_count={self._peak_count}, last peak={self._peaks[-1] if self._peaks else None}")

        # If there is enough peaks, calculate the amplitude of the induced oscillation on last 4 peaks.
        # If this amplitude is within the desired tolerance, the oscillation is stable and the algorithm
        # is finished.
        amplitude = 0
        if is_peak and (len(self._peaks) > 4):
            abs_max = self._peaks[-2].value
            abs_min = self._peaks[-2].value
            for i in range(0, len(self._peaks) - 2):
                amplitude += abs(self._peaks[i].value - self._peaks[i + 1].value)
                abs_max = max(self._peaks[i].value, abs_max)
                abs_min = min(self._peaks[i].value, abs_min)

            amplitude /= 6.0  # TODO

            # check convergence criterion for amplitude of induced oscillation
            amplitude_dev = ((0.5 * (abs_max - abs_min) - amplitude) / amplitude)
            if amplitude_dev < self._peak_amplitude_tolerance:
                self._state = self._state.DONE

        # If the oscillation is not stable even after certain amount of peaks, this algorithm
        # most likely cannot find the correct coefficients with the given parameters.
        if self._peak_count >= self._max_peaks:
            self._output = self._output_zero
            self._state = self._state.FAILED

        elif self._state is self._state.DONE:
            self._output = self._output_zero

            # Calculate the ultimate gain
            self._ku = 4.0 * self._output_step / (amplitude * math.pi)

            # Calculate the ultimate period in seconds
            period1 = self._peaks[3].timestamp - self._peaks[1].timestamp
            period2 = self._peaks[4].timestamp - self._peaks[2].timestamp
            self._pu = 0.5 * (period1 + period2)

        return self._output


def tune_pid():
    """Override the main Robot class just to tune the PID parameters"""
    import robot  # Must be local import to avoid circular import!

    class Robot(robot.Robot):
        def __init__(self, speed: bool = True, rotation: bool = False):
            super().__init__()
            ts_s = self.time_step / 1000.0

            # PID tunner for speed control
            if speed:
                self.speed_tuner = PIDTunner(
                    ts_s,
                    target_value=0.1, output_step=0.9 * self.drive.max_motor_velocity,
                    time_function=self.getTime
                )
            else:
                self.speed_tuner = None

            # PID tunner for rotation control
            if rotation:
                target_rotation = self.imu.yaw_dg + 90
                if target_rotation >= 360:
                    # Turn in the opposite direction
                    target_rotation -= 180
                    self.rotating_left = True
                else:
                    self.rotating_left = False

                self.rotation_tunner = PIDTunner(
                    ts_s,
                    target_value=target_rotation, output_step=0.9 * self.drive.max_motor_velocity,
                    time_function=self.getTime
                )
            else:
                self.rotation_tunner = None

        def run(self) -> bool:
            if not self.step():
                return False

            if self.speed_tuner:
                velocity = self.speed_tuner(self.gps.speed)
                self.drive.set_motor_velocity(velocity, velocity)

                if self.speed_tuner.completed:
                    print(f"Speed PID: {self.speed_tuner.parameters}")
                elif self.speed_tuner.failed:
                    print("Speed PID: Failed")

                if self.speed_tuner.done:
                    self.speed_tuner = None

            elif self.rotation_tunner:
                velocity = self.rotation_tunner(self.imu.yaw_dg)
                if self.rotating_left:
                    velocity = -velocity
                self.drive.set_motor_velocity(-velocity, velocity)

                if self.rotation_tunner.completed:
                    print(f"Speed PID: {self.speed_tuner.parameters}")
                elif self.rotation_tunner.failed:
                    print("Speed PID: Failed")

                if self.rotation_tunner.done:
                    self.rotation_tunner = None

            # Run the simulation until both PID tuners are done
            return bool(self.speed_tuner or self.rotation_tunner)

    robot = Robot()
    while robot.run():
        # Do nothing else
        pass


if __name__ == "__main__":
    tune_pid()
