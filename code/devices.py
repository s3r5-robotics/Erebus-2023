import inspect
import math
import sys
import typing
from typing import Tuple

import controller.device
import controller.sensor
from utils import InstanceSubclass


# Support all devices, and only devices from Erebus Robot Customisation 2023 webpage
# https://v23.robot.erebus.rcj.cloud/


class Device:
    """Base class for all devices"""

    def __init__(self, time_step: int) -> None:
        self.time_step = time_step


class Sensor(Device, controller.sensor.Sensor):
    """Base class for all sensors"""

    def __init__(self, time_step: int) -> None:
        Device.__init__(self, time_step)
        self.enable(time_step)


DeviceType = typing.TypeVar("DeviceType", Device, controller.device.Device)
"""
Type alias for any type which is subclass of Device or controller.Device

TypeVar is used in function typing instead of just the base-class to enable static type checking of the return type
of a function, based on the arguments.
https://medium.com/@steveYeah/using-generics-in-python-99010e5056eb
https://stackoverflow.com/questions/69184577/return-type-based-on-type-argument
https://stackoverflow.com/questions/68650071/type-annotations-for-base-and-inherited-classes-is-generic-and-typevar-the-rig
"""


# region Sensors

class Gyro(InstanceSubclass, Sensor, controller.Gyro):
    def __init__(self, _: controller.Gyro, time_step: int):
        Sensor.__init__(self, time_step)


class InertialUnit(InstanceSubclass, Sensor, controller.InertialUnit):
    def __init__(self, _: controller.InertialUnit, time_step: int):
        Sensor.__init__(self, time_step)

    @property
    def yaw_pitch_roll_dg(self) -> Tuple[float, float, float]:
        """Get yaw, pitch, roll in degrees"""
        # The same as (v * 180 / math.pi for v in self.roll_pitch_yaw) but better/shorter looking
        r, p, y = map(math.degrees, self.roll_pitch_yaw)
        return y, p, r

    @property
    def yaw_dg(self) -> float:
        """Get yaw in degrees"""
        return math.degrees(self.roll_pitch_yaw[2])


class GPS(InstanceSubclass, Sensor, controller.GPS):

    def __init__(self, _: controller.GPS, time_step: int):
        Sensor.__init__(self, time_step)


class Camera(InstanceSubclass, Sensor, controller.Camera):

    def __init__(self, _: controller.Camera, time_step: int):
        Sensor.__init__(self, time_step)


class ColorSensor(InstanceSubclass, Sensor, controller.Camera):

    def __init__(self, _: controller.Camera, time_step: int):
        Sensor.__init__(self, time_step)


class Accelerometer(InstanceSubclass, Sensor, controller.Accelerometer):

    def __init__(self, _: controller.Accelerometer, time_step: int):
        Sensor.__init__(self, time_step)


class Lidar(InstanceSubclass, Sensor, controller.Lidar):

    def __init__(self, _: controller.Lidar, time_step: int):
        Sensor.__init__(self, time_step)
        self.enablePointCloud()


# endregion Sensors

# region Distance Sensors

class DistanceSensor(InstanceSubclass, Sensor, controller.DistanceSensor):

    def __init__(self, _: controller.DistanceSensor, time_step: int):
        Sensor.__init__(self, time_step)


# endregion Distance Sensors

# region Wheels

class PositionSensor(InstanceSubclass, Sensor, controller.PositionSensor):

    def __init__(self, _: controller.PositionSensor, time_step: int):
        Sensor.__init__(self, time_step)


class Motor(InstanceSubclass, Device, controller.Motor):

    def __init__(self, _: controller.Motor, sensor: controller.PositionSensor, time_step: int):
        Device.__init__(self, time_step)
        # Prefer integrated sensor if available
        self.sensor = PositionSensor(self.position_sensor or sensor, time_step)
        # Enable velocity control instead of position control
        self.target_position = float("inf")
        self.target_velocity = 0


# endregion Wheels

# region Other, built-in

class LED(InstanceSubclass, Device, controller.LED):

    def __init__(self, _: controller.LED, time_step: int):
        Device.__init__(self, time_step)


class Receiver(InstanceSubclass, Sensor, controller.Receiver):

    def __init__(self, _: controller.Receiver, time_step: int):
        Sensor.__init__(self, time_step)


class Emitter(InstanceSubclass, Device, controller.Emitter):

    def __init__(self, _: controller.Emitter, time_step: int):
        Device.__init__(self, time_step)


# endregion Other, built-in


def _check_implementations():
    to_implement = set(cls[0] for cls in inspect.getmembers(controller) if inspect.isclass(cls[1]))
    implemented = set(cls[0] for cls in inspect.getmembers(sys.modules[__name__]) if inspect.isclass(cls[1]))
    missing = to_implement - implemented
    if missing:
        print("Missing implementations for the following controller devices:", ", ".join(missing))


if __name__ == "__main__":
    _check_implementations()
