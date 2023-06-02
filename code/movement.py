from typing import Optional

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

        # Motor velocity is set in angular velocity (rotation speed in radians per second), in limited range
        self._max_motor_velocity = Angle(min(self._ml.max_velocity, self._mr.max_velocity), normalize=None)

    def _clamp_motor_velocity(self, velocity: Angle) -> Angle:
        return Angle(min(max(velocity, -self._max_motor_velocity), self._max_motor_velocity), normalize=None)

    def set_motor_velocity(self, left_or_both: Angle, right: Angle = None) -> None:
        """Set target angular velocity in rad/s (ω) for both wheels or each wheel separately"""
        self._ml.velocity = self._clamp_motor_velocity(left_or_both)
        self._mr.velocity = self._clamp_motor_velocity(right if (right is not None) else left_or_both)

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
        """Current rotation angle"""
        return self._imu.yaw

    @rotation.setter
    def rotation(self, rotation: Angle) -> None:
        """Set new target rotation angle"""
        self.target_rotation = rotation

    def __call__(self) -> None:
        """Update internal states - shall be called once every timestep"""
        velocity = self.velocity
        rotation = self.rotation

        # Fix for the first call (rotation cannot be retrieved in __init__ because no simulation step has been done yet)
        if self.target_rotation is None:
            self.target_rotation = rotation

        # TODO: Calculate motor angular velocities (rotating speed) and set them

        if debug.MOVEMENT:
            yaw, pitch, roll = self._imu.yaw_pitch_roll
            mlv, mrv = self._ml.velocity, self._mr.velocity
            print(f"Y|P|R  {yaw.deg:.3f} | {pitch.deg:.3f} | {roll.deg:.3f}", end="    ")
            print(f"av|tv  {velocity:.6f} | {self.target_velocity:.3f}", end="    ")
            print(f"ar|tr  {rotation.deg:.3f} | {self.target_rotation.deg:.3f}", end="    ")
            print(f"ml|mr  {mlv:.3f}/{mlv.deg:.3f} | {mrv:.3f}/{mrv.deg:.3f}", end="    ")
