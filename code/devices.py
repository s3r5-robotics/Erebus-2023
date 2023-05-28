import inspect
import sys

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


# region Sensors

class Gyro(InstanceSubclass, Sensor, controller.Gyro):
    def __init__(self, _: controller.Gyro, time_step: int):
        Sensor.__init__(self, time_step)


class InertialUnit(InstanceSubclass, Sensor, controller.InertialUnit):
    def __init__(self, _: controller.InertialUnit, time_step: int):
        Sensor.__init__(self, time_step)


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
